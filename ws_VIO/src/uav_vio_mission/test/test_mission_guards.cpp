#include <gtest/gtest.h>
#include <utility>
#include "uav_vio_mission/mission_context.hpp"

using namespace uav_vio_mission;
TEST(Readiness, RequiresStableFreshStreamsAndResetsOnGapsEvenWithoutTimerTick)
{
  StableReadiness gate(2.0);
  EXPECT_FALSE(gate.ready(0.0));
  for (int i = 0; i <= 21; ++i) { gate.vio(i * 0.1, true); gate.local(i * 0.1, true); }
  EXPECT_TRUE(gate.ready(2.1));
  EXPECT_FALSE(gate.ready(2.6));
  // A fresh delivery after a silent gap must not retain the previous stable epoch.
  gate.vio(3.0, true); gate.local(3.0, true);
  EXPECT_FALSE(gate.ready(3.0));
  for (int i = 1; i <= 21; ++i) { gate.vio(3.0 + i * 0.1, true); gate.local(3.0 + i * 0.1, true); }
  EXPECT_TRUE(gate.ready(5.1));
  gate.vio(5.2, false);
  EXPECT_FALSE(gate.ready(5.2));
  gate.vio(5.3, true);
  EXPECT_FALSE(gate.ready(5.3));
}
TEST(Readiness, InvalidLocalAndTimeReversalCannotPassTheGate)
{
  StableReadiness gate(0.1);
  gate.vio(10.0, true); gate.local(10.0, true);
  EXPECT_TRUE(gate.ready(10.2));
  gate.local(10.3, false);
  EXPECT_FALSE(gate.ready(10.3));
  EXPECT_FALSE(gate.ready(9.0));
}
TEST(ActionLifecycle, LateCallbackCannotCompleteOrCancelNewInvocation)
{
  InvocationGuard guard;
  const auto old_token = guard.begin();
  int completion_count = 0;
  if (guard.finish(old_token)) ++completion_count;
  EXPECT_FALSE(guard.finish(old_token));
  const auto next_token = guard.begin();
  // Same guard check used at the top of LocalAction::tick: an invalid token
  // returns before it can touch the current invocation's timer or callback.
  int active_timer_cancel_count = 0;
  const auto queued_old_tick = [&] {
    if (!guard.valid(old_token)) return;
    ++active_timer_cancel_count;
  };
  queued_old_tick();
  EXPECT_EQ(active_timer_cancel_count, 0);
  EXPECT_TRUE(guard.valid(next_token));
  EXPECT_FALSE(guard.finish(old_token));
  if (guard.finish(next_token)) ++completion_count;
  EXPECT_EQ(completion_count, 2);
  const auto canceled = guard.begin(); guard.cancel();
  EXPECT_FALSE(guard.finish(canceled));
}
TEST(LocalFrame, EveryRelevantResetAndReferenceChangeInvalidatesTheEpoch)
{
  const FrameEpoch captured{1, 2, 3, 4, 5, 1234};
  EXPECT_EQ(captured, (FrameEpoch{1, 2, 3, 4, 5, 1234}));
  for (int field = 0; field < 6; ++field) {
    auto changed = captured;
    switch (field) {
      case 0: ++changed.xy; break; case 1: ++changed.z; break;
      case 2: ++changed.vxy; break; case 3: ++changed.vz; break;
      case 4: ++changed.heading; break; case 5: ++changed.reference; break;
    }
    EXPECT_NE(captured, changed);
  }
}
TEST(InactiveUpdates, AcceptsOnlyFreshInactiveLandedDisarmedState)
{
  EXPECT_TRUE(canReplaceMission(false, true, false, true));
  EXPECT_FALSE(canReplaceMission(true, true, false, true));
  EXPECT_FALSE(canReplaceMission(false, false, false, true));
  EXPECT_FALSE(canReplaceMission(false, true, true, true));
  EXPECT_FALSE(canReplaceMission(false, true, false, false));
}
TEST(LandingObserver, SuccessIndependentOfUpstreamCallbackOrdering)
{
  const auto success = [](bool landed, bool armed, bool failed = false, bool interrupted = false) {
    return terminalLandingSuccess(true, true, failed, interrupted, landed, armed, true);
  };
  EXPECT_FALSE(success(false, true));
  EXPECT_FALSE(success(true, true));  // mode-complete before disarm
  EXPECT_TRUE(success(true, false)); // disarm may deactivate before mode-complete
  EXPECT_FALSE(success(true, false, true));
  EXPECT_FALSE(success(true, false, false, true));
  EXPECT_FALSE(terminalLandingSuccess(false, true, false, false, true, false, true));
  EXPECT_FALSE(terminalLandingSuccess(true, false, false, false, true, false, true));
  EXPECT_FALSE(terminalLandingSuccess(true, true, false, false, true, false, false));
}
TEST(LandingObserver, NativeDisarmRestoresOwnedModeBeforeDeactivationCallback)
{
  using Status = px4_msgs::msg::VehicleStatus;
  constexpr uint8_t owned_mode = 25;
  // Replay the observed SITL order. The executor is still active when the
  // disarmed status restores mode 25 after Takeoff, LocalGoto and native Land.
  const std::pair<uint8_t, uint8_t> sequence[] = {
    {25, Status::ARMING_STATE_ARMED}, {17, Status::ARMING_STATE_ARMED},
    {24, Status::ARMING_STATE_ARMED}, {18, Status::ARMING_STATE_ARMED},
    {25, Status::ARMING_STATE_DISARMED},
  };
  bool active = true, seen_flight_mode = false, failed = false;
  bool final_land = false, upstream_completed = false;
  for (const auto& [nav_state, arming_state] : sequence) {
    Status incoming;
    incoming.nav_state = nav_state;
    incoming.arming_state = arming_state;
    if (nav_state == 18) { final_land = true; upstream_completed = true; }
    ASSERT_TRUE(active);  // onDeactivated has not run yet.
    const bool incoming_owned_mode = incoming.nav_state == owned_mode;
    if (!incoming_owned_mode) seen_flight_mode = true;
    if (ownedModeReselectedWhileArmed(
        incoming.arming_state == Status::ARMING_STATE_ARMED,
        seen_flight_mode, incoming_owned_mode)) failed = true;
    EXPECT_FALSE(failed) << "nav_state=" << int(nav_state) << " arming_state=" << int(arming_state);
  }
  ASSERT_TRUE(seen_flight_mode);
  ASSERT_TRUE(upstream_completed);
  EXPECT_TRUE(terminalLandingSuccess(true, final_land, failed, false, true, false, true));
  EXPECT_FALSE(terminalLandingSuccess(true, final_land, failed, false, false, false, true));
  EXPECT_FALSE(terminalLandingSuccess(true, final_land, failed, false, true, false, false));
}
TEST(LandingObserver, ArmedOwnedModeReselectionStillFailsAfterFlightAndLandCompletion)
{
  using Status = px4_msgs::msg::VehicleStatus;
  struct FlightState { uint8_t nav_state; bool final_land; bool upstream_completed; };
  const FlightState prior_states[] = {
    {17, false, false}, {24, false, false}, {18, true, false}, {18, true, true},
  };
  for (const auto& prior : prior_states) {
    SCOPED_TRACE(::testing::Message() << "previous mode=" << int(prior.nav_state)
        << " final_land=" << prior.final_land << " upstream_completed=" << prior.upstream_completed);
    Status incoming;
    incoming.nav_state = 25;
    incoming.arming_state = Status::ARMING_STATE_ARMED;
    const bool failed = ownedModeReselectedWhileArmed(
        incoming.arming_state == Status::ARMING_STATE_ARMED,
        prior.nav_state != 25, incoming.nav_state == 25);
    EXPECT_TRUE(failed);
    // A subsequent disarm cannot turn an already failed flight into success.
    EXPECT_FALSE(terminalLandingSuccess(true, prior.final_land, failed, false, true, false, true));
  }
}
TEST(ClockContract, SimulationProgressAndSteadyWatchdogHaveSeparateDeadlines)
{
  const PhaseDeadline deadline{10.0, 100.0, 45.0};
  EXPECT_FALSE(deadline.expired(30.0, 145.0)); // low real-time factor, only 20 flight seconds
  EXPECT_TRUE(deadline.expired(55.0, 160.0));
  EXPECT_TRUE(deadline.expired(10.0, 290.0));  // stalled sim still bounded at 4x +10
  EXPECT_TRUE(deadline.expired(9.0, 101.0));  // clock reset fails closed
}

TEST(InitialFrame, RebaseStartupAlignmentAndFreezeOnlyAfterStableEpoch)
{
  InitialFrameState initialization;
  const FrameEpoch initial{1, 2, 3, 4, 5, 100};
  initialization.capture(initial, 0.25, 10.0);
  auto aligned = initial; ++aligned.heading; ++aligned.vxy;
  EXPECT_EQ(initialization.observe(aligned, 0.0, 11.0), InitialFrameState::Update::Retarget);
  EXPECT_FALSE(initialization.stable(13.9));
  EXPECT_TRUE(initialization.stable(14.0));
  ++aligned.xy;
  EXPECT_EQ(initialization.observe(aligned, 0.0, 14.1), InitialFrameState::Update::Retarget);
  EXPECT_FALSE(initialization.stable(14.1));
  EXPECT_DOUBLE_EQ(initialization.groundZ(), 0.25);
}
TEST(InitialFrame, AdjustsGroundZExactlyOnceAndHandlesContiguousCounterWrap)
{
  InitialFrameState initialization;
  FrameEpoch epoch{255, 255, 255, 255, 255, 100};
  initialization.capture(epoch, 0.25, 10.0);
  epoch = {0, 0, 0, 0, 0, 100};
  EXPECT_EQ(initialization.observe(epoch, -0.4, 11.0), InitialFrameState::Update::Retarget);
  EXPECT_NEAR(initialization.groundZ(), -0.15, 1e-9);
  EXPECT_EQ(initialization.observe(epoch, -0.4, 11.1), InitialFrameState::Update::Unchanged);
  EXPECT_NEAR(initialization.groundZ(), -0.15, 1e-9);
}
TEST(InitialFrame, RejectsDroppedResetsReferenceChangesNonfiniteShiftAndClockReset)
{
  InitialFrameState initialization;
  const FrameEpoch epoch{1, 2, 3, 4, 5, 100};
  initialization.capture(epoch, 0.0, 10.0);
  auto changed = epoch; changed.xy += 2;
  EXPECT_EQ(initialization.observe(changed, 0.0, 11.0), InitialFrameState::Update::Invalid);
  changed = epoch; ++changed.reference;
  EXPECT_EQ(initialization.observe(changed, 0.0, 11.0), InitialFrameState::Update::Invalid);
  changed = epoch; ++changed.z;
  EXPECT_EQ(initialization.observe(changed, NAN, 11.0), InitialFrameState::Update::Invalid);
  EXPECT_EQ(initialization.observe(epoch, 0.0, 9.0), InitialFrameState::Update::Invalid);
}
