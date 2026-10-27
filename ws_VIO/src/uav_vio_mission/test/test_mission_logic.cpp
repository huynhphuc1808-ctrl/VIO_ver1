#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "uav_vio_mission/mission_logic.hpp"

TEST(MissionParameters, AcceptsRequiredRangesAndRejectsInvalidValues)
{
  uav_vio_mission::MissionParameters parameters;
  EXPECT_NO_THROW(parameters.validate());
  parameters.radius_m = 0.99;
  EXPECT_THROW(parameters.validate(), std::invalid_argument);
  parameters.radius_m = 1.0;
  parameters.altitude_m = 0.99;
  EXPECT_THROW(parameters.validate(), std::invalid_argument);
  parameters.altitude_m = 1.0;
  EXPECT_NO_THROW(parameters.validate());
  parameters.altitude_m = 3.0;
  EXPECT_NO_THROW(parameters.validate());
  parameters.speed_m_s = NAN;
  EXPECT_THROW(parameters.validate(), std::invalid_argument);
  parameters.speed_m_s = 1.0;
  parameters.laps = 0;
  EXPECT_THROW(parameters.validate(), std::invalid_argument);
}

TEST(PhaseDeadlines, AltitudeGotoScalesAndRemainsBounded)
{
  EXPECT_DOUBLE_EQ(uav_vio_mission::altitudeGotoDeadlineSeconds(1.0), 60.0);
  EXPECT_DOUBLE_EQ(uav_vio_mission::altitudeGotoDeadlineSeconds(2.75), 67.0);
  EXPECT_DOUBLE_EQ(uav_vio_mission::altitudeGotoDeadlineSeconds(100.0), 300.0);
}

TEST(PhaseDeadlines, OrbitUsesExpectedLapTimeWithMinimumAndMaximum)
{
  EXPECT_DOUBLE_EQ(uav_vio_mission::orbitDeadlineSeconds(1.0, 1, 1.0), 60.0);
  EXPECT_NEAR(uav_vio_mission::orbitDeadlineSeconds(10.0, 2, 1.0),
              2.0 * 2.0 * M_PI * 10.0 * 2.0 + 30.0, 1e-9);
  EXPECT_DOUBLE_EQ(uav_vio_mission::orbitDeadlineSeconds(100.0, 10, 1.0), 900.0);
}

TEST(PhaseDeadlines, HoldIncludesMarginAndHasFiniteCap)
{
  EXPECT_DOUBLE_EQ(uav_vio_mission::holdDeadlineSeconds(0.0), 15.0);
  EXPECT_DOUBLE_EQ(uav_vio_mission::holdDeadlineSeconds(2.0), 17.0);
  EXPECT_DOUBLE_EQ(uav_vio_mission::holdDeadlineSeconds(5000.0), 3600.0);
}

TEST(LapTracker, WaitsForRadialAcquisitionAndUnwrapsClockwiseWrap)
{
  uav_vio_mission::LapTracker tracker(0.0, 0.0, 2.0, true);
  tracker.update(0.0, 0.0, 0.2);
  EXPECT_FALSE(tracker.acquired());
  tracker.update(2.0 * std::cos(3.0), 2.0 * std::sin(3.0), 0.2);
  tracker.update(2.0 * std::cos(-3.0), 2.0 * std::sin(-3.0), 0.2);
  EXPECT_NEAR(tracker.angle(), 2.0 * M_PI - 6.0, 1e-6);
}

TEST(LapTracker, CounterclockwiseAccumulationHasPositiveProgress)
{
  uav_vio_mission::LapTracker tracker(1.0, -2.0, 1.0, false);
  tracker.update(2.0, -2.0, 0.1);
  tracker.update(1.0, -3.0, 0.1);
  tracker.update(0.0, -2.0, 0.1);
  EXPECT_NEAR(tracker.angle(), M_PI, 1e-6);
}

TEST(LapTracker, IgnoresSamplesOutsideRadialBand)
{
  uav_vio_mission::LapTracker tracker(0.0, 0.0, 1.0, true);
  tracker.update(1.0, 0.0, 0.1);
  tracker.update(0.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(tracker.angle(), 0.0);
}
