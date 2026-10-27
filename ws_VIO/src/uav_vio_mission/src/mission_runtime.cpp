#include "uav_vio_mission/mission_runtime.hpp"
#include "uav_vio_mission/mission_actions.hpp"
#include "uav_vio_mission/mission_definition.hpp"
#include "uav_vio_mission/mission_file_observer.hpp"

#include <filesystem>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>

namespace uav_vio_mission {
namespace {
using namespace std::chrono_literals;
class LocalMissionExecutor final : public px4_ros2::MissionExecutor {
 public:
  using MissionExecutor::MissionExecutor;
  px4_ros2::ModeBase::ModeID ownedModeId() const { return modeId(); }
  px4_ros2::ModeExecutorBase& executor() { return modeExecutor(); }
 protected:
  bool doRegisterImpl(MissionMode& mode, MissionModeExecutor& executor) override {
    if (!MissionExecutor::doRegisterImpl(mode, executor)) return false;
    // Registration sets the rate from the declared setpoint type, so disable
    // owned-mode publishing only after registration has completed.
    mode.setSetpointUpdateRate(0.0F);
    return true;
  }
};

MissionParameters circleParameters(rclcpp::Node& node)
{
  MissionParameters p;
  p.radius_m = node.declare_parameter<double>("radius_m", 2.0);
  p.altitude_m = node.declare_parameter<double>("altitude_m", 2.0);
  p.speed_m_s = node.declare_parameter<double>("speed_m_s", 1.0);
  const auto direction = node.declare_parameter<std::string>("direction", "clockwise");
  if (direction != "clockwise" && direction != "counterclockwise") throw std::invalid_argument("invalid direction parameter");
  p.clockwise = direction == "clockwise";
  p.laps = node.declare_parameter<int>("laps", 1);
  p.hold_seconds = node.declare_parameter<double>("hold_seconds", 2.0);
  p.radial_tolerance_m = node.declare_parameter<double>("radial_tolerance_m", 0.5);
  p.gate_stable_seconds = node.declare_parameter<double>("gate_stable_seconds", 2.0);
  p.auto_start = node.declare_parameter<bool>("auto_start", false);
  p.validate();
  return p;
}
}  // namespace

struct MissionRuntime::Impl {
  explicit Impl(std::shared_ptr<rclcpp::Node> n) : node(std::move(n)) {
    parameters = circleParameters(*node);
    mission_file = node->declare_parameter<std::string>("mission_file", "");
    // Validate all flight intent before constructing publishers/command endpoints.
    definition = mission_file.empty() ? validateMission(circleMission(parameters)) : loadMission(mission_file);
    if (!mission_file.empty()) mission_file = std::filesystem::absolute(mission_file).string();
    context = std::make_shared<MissionContext>(*node, parameters.gate_stable_seconds);
    context->mission_valid = true;
    context->goto_mode = std::make_shared<LocalGotoMode>(*node, context);
    mission = std::make_unique<LocalMissionExecutor>("VIO Mission", localMissionConfiguration(context), *node);
    context->command_owner = [this] { return commandOwner(); };
    context->actions_allowed = [this] {
      return active && !failed && !interrupted &&
          (!seen_flight_mode || context->vehicle_status.navState() != mission->ownedModeId());
    };
    context->vehicle_status.onUpdate([this](const px4_msgs::msg::VehicleStatus& status) {
      if (!active || !started || failed) return;
      const bool owned_mode = status.nav_state == mission->ownedModeId();
      if (!owned_mode) seen_flight_mode = true;
      if (ownedModeReselectedWhileArmed(
          status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
          seen_flight_mode, owned_mode)) {
        fail("owned mission mode reselected in flight; resume disabled");
      }
    });
    context->native_command = [this](uint32_t command, float first, float second) {
      if (!commandOwner() || !context->actions_allowed() || !context->armed()) return px4_ros2::Result::Deactivated;
      return mission->executor().sendCommandSync(command, first, second, NAN, NAN, NAN, NAN, NAN);
    };
    context->fail = [this](const std::string& reason) { fail(reason); };
    context->activity = [this](const std::string& action_name, const std::string& state) {
      action = action_name;
      if (action_name == "onFailure") {
        failed = true;
        context->initializing = false;
        terminal = true;
        phase = "failed";
        if (error.empty()) error = "PX4 mission abort: " + state;
      } else phase = state;
    };
    mission->onActivated([this] { activated(); });
    mission->onDeactivated([this] { deactivated(); });
    mission->onProgressUpdate([this](int item_index) {
      if (!active || failed || interrupted) return;
      index = item_index;
      if (definition.mission.indexValid(index)) {
        action = std::get<px4_ros2::ActionItem>(definition.mission.items()[index]).name;
        if (action == "land") {
          final_land = true; phase = "landing";
          context->failure_land_requested = true;
          landing_deadline = PhaseDeadline{node->get_clock()->now().seconds(), monotonicSeconds(), 120.0};
        }
      }
    });
    mission->onCompleted([this] {
      if (!started || !final_land || failed || interrupted) return;
      upstream_completed = true;
      // Completion of a native mode can precede disarm; status observation is
      // authoritative for terminal success in either callback ordering.
      if (!failed && !interrupted) phase = "landing";
    });
    mission->onReadynessUpdate([this](bool ready, const std::vector<std::string>&) { framework_ready = ready; });
    status_publisher = node->create_publisher<std_msgs::msg::String>(
        "/uav_vio/mission/status", rclcpp::QoS(1).reliable().transient_local());
    arming_checks = std::make_unique<px4_ros2::HealthAndArmingChecks>(*node,
      [this](px4_ros2::HealthAndArmingCheckReporter& reporter) {
        // No inflight health restriction on native Land. PX4 owns its failsafes.
        if (context->armed()) return;
        if (!context->gateReady() || terminal) {
          reporter.armingCheckFailureExt(px4_ros2::events::ID("vio_mission_prearm_not_ready"),
             px4_ros2::events::Log::Error, "VIO local mission is not ready to arm");
        }
      });
    mission->setMission(definition.mission);
    publishStatus();
  }
  ~Impl() {
    file_monitor.reset();
    timer.reset();
    // Actions/factories own the context. Clear back-references to this runtime
    // before destroying the executor and its callback cancellation paths.
    context->fail = [](const std::string&) {};
    context->command_owner = [] { return false; };
    context->actions_allowed = [] { return false; };
    context->activity = [](const std::string&, const std::string&) {};
    mission.reset();
    context->goto_mode.reset();
  }
  void start() {
    if (!px4_ros2::waitForFMU(*node, 45s, 5s)) throw std::runtime_error("PX4 FMU unavailable");
    if (!context->goto_mode->doRegister()) throw std::runtime_error("local Goto mode registration failed");
    if (!mission->doRegister()) throw std::runtime_error("mission executor registration failed");
    if (!arming_checks->doRegister("VIO Mission Gate")) throw std::runtime_error("VIO prearm check registration failed");
    bootstrap_sender = std::make_unique<px4_ros2::VehicleCommandSender>(*node, "");
    registered = true;
    if (!mission_file.empty()) {
      file_monitor = std::make_unique<px4_ros2::MissionFileMonitor>(node, mission_file,
        [this, initial = true](const std::shared_ptr<px4_ros2::Mission>& candidate) mutable {
          // The monitor announces the existing file once on startup. It is
          // already strictly loaded before endpoint creation; no replacement
          // should be attempted while initial telemetry is still warming up.
          if (initial) {
            initial = false;
            if (candidate->checksum() == definition.revision) return;
          }
          updateMission(candidate);
        });
      file_error_observer = std::make_unique<MissionFileErrorObserver>(mission_file);
    }
    timer = node->create_wall_timer(100ms, [this] { tick(); });
    RCLCPP_INFO(node->get_logger(), "VIO Mission registered: mode=%u local_mode=%u executor=%d revision=%s", mission->ownedModeId(), context->goto_mode->id(), mission->executor().id(), definition.revision.c_str());
    publishStatus();
  }
  bool commandOwner() const {
    return registered && context->statusFresh() && mission->executor().isInCharge() &&
           context->vehicle_status.last().executor_in_charge == mission->executor().id();
  }
  bool selected() const {
    if (!registered || !context->statusFresh()) return false;
    const auto& status = context->vehicle_status.last();
    return status.executor_in_charge == mission->executor().id() &&
           status.nav_state_user_intention == mission->ownedModeId();
  }
  void activated() {
    active = true;
    if (pending_reset || !prepared || !arm_sent || started || terminal ||
        !context->readiness.fresh(monotonicSeconds()) || !context->origin_epoch ||
        *context->origin_epoch != context->epoch()) {
      context->failure_land_requested = false; // explicit new ownership episode
      if (failed) mission->abort();
      else fail("mission activation has no fresh authorized ground start; resume is disabled");
      return;
    }
    started = true;
    phase = "running";
    RCLCPP_INFO(node->get_logger(), "Mission activated from provisional local ground reference (%.2f, %.2f, %.2f)", context->origin.x(), context->origin.y(), context->origin.z());
  }
  void deactivated() {
    active = false;
    pending_reset = true;
    deactivation_pending = true;
    deactivation_received = monotonicSeconds();
    // Let all shared VehicleStatus subscribers observe the same message first.
    // A final Land/disarm can deactivate upstream before waitUntilDisarmed.
  }
  void fail(const std::string& reason) {
    if (failed) return;
    failed = true;
    terminal = true;
    error = reason;
    context->initializing = false;
    phase = "failed";
    RCLCPP_ERROR(node->get_logger(), "VIO mission failed: %s", reason.c_str());
    if (active) mission->abort();
  }
  void updateMission(const std::shared_ptr<px4_ros2::Mission>& candidate) {
    // Never defer an active write for a later flight. The file must be written
    // again after the vehicle is inactive, landed and disarmed.
    if (!canReplaceMission(active, context->landed(), context->armed(), context->vehicleFresh()) ||
        (prepared && !terminal)) {
      update_error = "mission update rejected: require inactive, landed, disarmed and no bootstrap in progress";
      RCLCPP_WARN(node->get_logger(), "%s", update_error.c_str());
      return;
    }
    try {
      auto replacement = loadMission(mission_file);
      if (replacement.revision != candidate->checksum()) throw std::invalid_argument("mission file changed during validation; write it again");
      if (!terminal && replacement.revision == definition.revision) {
        // An explicit inactive rewrite can restore the already loaded valid
        // definition after a malformed file. Clear its rejection without
        // resetting mission state or granting a new bootstrap episode.
        update_error.clear();
        return;
      }
      mission->resetMission();
      definition = std::move(replacement);
      resetForNewDefinition();
      context->mission_valid = true;
      mission->setMission(definition.mission);
      RCLCPP_INFO(node->get_logger(), "Accepted inactive mission revision %s", definition.revision.c_str());
    } catch (const std::exception& exception) {
      update_error = exception.what();
      RCLCPP_ERROR(node->get_logger(), "Rejected mission update: %s", exception.what());
    }
  }
  void resetForNewDefinition() {
    started = false; prepared = false; arm_sent = false; terminal = false;
    failed = false; interrupted = false; success = false; final_land = false;
    upstream_completed = false; pending_reset = false; deactivation_pending = false; seen_flight_mode = false;
    context->failure_land_requested = false;
    context->origin_epoch.reset();
    context->frame_frozen = false; context->initializing = false;
    manual_selection_eligible = false;
    landing_deadline.reset();
    phase = "waiting_ready"; action.clear(); error.clear(); update_error.clear(); index = -1;
    // auto_consumed intentionally persists for the lifetime of this process.
  }
  void prepare() {
    if (!context->gateReady() || !framework_ready || context->armed() || !context->landed()) return;
    context->captureOrigin();
    prepared = true;
    bootstrap_started = monotonicSeconds();
  }
  void tick() {
    if (file_error_observer) {
      const auto observation = file_error_observer->observe();
      if (observation && !observation->error.empty()) {
        update_error = observation->error;
        RCLCPP_ERROR(node->get_logger(), "Rejected mission file observation: %s", update_error.c_str());
      }
      // Valid content is applied only by the MissionFileMonitor callback.
      // In particular, it must not clear a rejection made during an active run.
    }
    context->readiness.observe(monotonicSeconds());
    if (deactivation_pending) {
      const bool landing_success = terminalLandingSuccess(started, final_land, failed, interrupted,
          context->landed(), context->armed(), context->vehicleFresh());
      // LandDetected is a separate low-rate topic. Reconcile disarm and landed
      // for five wall seconds; an armed ownership takeover is immediately final.
      const bool wait_for_landed = started && final_land && !failed && !landing_success &&
          (!context->statusFresh() || !context->armed()) &&
          monotonicSeconds() - deactivation_received < 5.0;
      if (!wait_for_landed) {
        deactivation_pending = false;
        if (started && !landing_success) {
          interrupted = true; terminal = true;
          if (!failed) { phase = "interrupted"; error = "mission deactivated; automatic resume disabled"; }
        }
      }
    }
    if (terminalLandingSuccess(started, final_land, failed, interrupted,
                               context->landed(), context->armed(), context->vehicleFresh())) {
      if (!success) RCLCPP_INFO(node->get_logger(), "VIO mission landed and disarmed successfully");
      success = true; terminal = true; phase = "completed";
    }
    if (pending_reset && !active) {
      mission->resetMission();
      context->mission_valid = false;
      prepared = false;
      arm_sent = false;
      context->initializing = false;
      pending_reset = false;
    }
    if (active && !failed && !context->statusFresh()) fail("PX4 vehicle status became stale; command ownership cannot be verified");
    if (active && !failed && !final_land && !context->flightHealthy()) fail("VIO/local state stale, invalid or frame epoch changed");
    if (final_land && !terminal && landing_deadline &&
        landing_deadline->expired(node->get_clock()->now().seconds(), monotonicSeconds())) {
      fail("native Land did not reach landed/disarmed state within bounded deadline; PX4 retains Land");
    }
    if (prepared && !active && !terminal && monotonicSeconds() - bootstrap_started > 30.0) {
      fail("mission bootstrap ownership/arming timeout");
    }
    if (!active && !terminal && !context->armed()) bootstrap();
    publishStatus();
  }
  void bootstrap() {
    const bool currently_selected = selected();
    if (context->statusFresh() && !currently_selected) manual_selection_eligible = true;
    if (!prepared) {
      if (parameters.auto_start && !auto_consumed && context->gateReady() && framework_ready && context->landed()) {
        prepare();
        if (!prepared) return;
        auto_consumed = true;
        phase = "selecting_mode";
        px4_msgs::msg::VehicleCommand command{};
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
        command.param1 = mission->ownedModeId();
        command.source_component = 1;  // normal command channel, before ownership exists
        command.target_system = 1;
        command.target_component = 1;
        command.from_external = true;
        if (bootstrap_sender->sendCommandSync(command) != px4_ros2::Result::Success) {
          fail("PX4 rejected mission-mode bootstrap selection");
          return;
        }
      } else if (!parameters.auto_start && currently_selected && manual_selection_eligible) {
        prepare();
        if (prepared) manual_selection_eligible = false;
      }
    }
    if (!prepared || terminal || arm_sent) return;
    if (monotonicSeconds() - bootstrap_started > 30.0) return fail("mission bootstrap ownership/readiness timeout");
    if (!selected()) return;
    if (!context->gateReady() || !framework_ready) return;
    if (!context->localFresh() || !context->origin_epoch || *context->origin_epoch != context->epoch()) {
      return fail("local frame reset before arming; load a fresh mission");
    }
    if (!context->vehicle_status.last().pre_flight_checks_pass) return;
    // Raw executor_in_charge is authoritative while disarmed. isInCharge() is
    // false until armed because upstream has ActivateOnlyWhenArmed policy.
    arm_sent = true;
    phase = "arming";
    mission->executor().arm([this](px4_ros2::Result result) {
      if (result == px4_ros2::Result::Success || result == px4_ros2::Result::Deactivated) return;
      fail("native PX4 arm request failed: " + std::string(px4_ros2::resultToString(result)));
    });
  }
  void publishStatus() {
    nlohmann::json value = {
      {"revision", definition.revision}, {"phase", phase}, {"action", action}, {"index", index},
      {"registered", registered}, {"mode_id", registered ? int(mission->ownedModeId()) : -1},
      {"local_mode_id", registered ? int(context->goto_mode->id()) : -1},
      {"executor_id", registered ? mission->executor().id() : -1},
      {"executor_in_charge", context->status_received ? int(context->vehicle_status.last().executor_in_charge) : -1},
      {"command_owner", commandOwner()}, {"armed", context->armed()}, {"landed", context->landed()},
      {"active", active}, {"started", started}, {"success", success}, {"terminal", terminal},
      {"error", error}, {"update_error", update_error}, {"mission_valid", context->mission_valid},
      {"ready", context->gateReady() && framework_ready}, {"frame_valid", context->frameValid()},
      {"frame_initializing", context->initializing},
      {"clock_ready", node->get_clock()->now().nanoseconds() > 0},
      {"origin_ned", context->origin_epoch ? nlohmann::json::array({context->origin.x(), context->origin.y(), context->origin.z()}) : nlohmann::json(nullptr)},
      {"upstream_completed", upstream_completed}, {"use_sim_time", node->get_parameter("use_sim_time").as_bool()}
    };
    std_msgs::msg::String message;
    message.data = value.dump();
    status_publisher->publish(message);
  }

  std::shared_ptr<rclcpp::Node> node;
  MissionParameters parameters;
  std::string mission_file;
  MissionDefinition definition;
  std::shared_ptr<MissionContext> context;
  std::unique_ptr<LocalMissionExecutor> mission;
  std::unique_ptr<px4_ros2::VehicleCommandSender> bootstrap_sender;
  std::unique_ptr<px4_ros2::HealthAndArmingChecks> arming_checks;
  std::unique_ptr<px4_ros2::MissionFileMonitor> file_monitor;
  std::unique_ptr<MissionFileErrorObserver> file_error_observer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
  rclcpp::TimerBase::SharedPtr timer;
  bool registered{false}, active{false}, started{false}, prepared{false}, arm_sent{false};
  bool auto_consumed{false}, framework_ready{false}, terminal{false}, failed{false};
  bool interrupted{false}, success{false}, final_land{false}, upstream_completed{false};
  bool pending_reset{false}, deactivation_pending{false}, seen_flight_mode{false};
  int index{-1};
  std::optional<PhaseDeadline> landing_deadline;
  double bootstrap_started{0.0}, deactivation_received{0.0};
  bool manual_selection_eligible{false};
  std::string phase{"waiting_ready"}, action, error, update_error;
};

MissionRuntime::MissionRuntime(std::shared_ptr<rclcpp::Node> node) : _impl(std::make_unique<Impl>(std::move(node))) {}
MissionRuntime::~MissionRuntime() = default;
void MissionRuntime::start() { _impl->start(); }
}  // namespace uav_vio_mission
