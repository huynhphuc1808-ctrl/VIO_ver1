#include "uav_vio_mission/mission_actions.hpp"
#include "uav_vio_mission/mission_logic.hpp"

#include <cmath>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

namespace uav_vio_mission {
namespace {
using namespace std::chrono_literals;

// MissionExecutor needs a registered setpoint type on its owned mode. This
// adapter intentionally never publishes: all actions activate a native or local
// mode. No global-position dependency is constructed.
class ActionOnlyTrajectory final : public px4_ros2::TrajectoryExecutorInterface {
 public:
  explicit ActionOnlyTrajectory(px4_ros2::ModeBase& mode)
      : _registration_type(std::make_shared<px4_ros2::MulticopterGotoSetpointType>(mode)) {}
  bool navigationItemTypeSupported(px4_ros2::NavigationItemType) override { return false; }
  bool frameSupported(px4_ros2::MissionFrame) override { return false; }
  void runTrajectory(const TrajectoryConfig& config) override { if (config.on_failure) config.on_failure(); }
  void updateSetpoint() override {}
 private:
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _registration_type;
};

class LocalAction final : public px4_ros2::ActionInterface {
 public:
  LocalAction(std::string action_name, const std::shared_ptr<MissionContext>& context)
      : _name(std::move(action_name)), _context(context) {}
  std::string name() const override { return _name; }
  bool canRun(const px4_ros2::ActionArguments&, std::vector<std::string>& errors) override {
    if (_context->gateReady()) return true;
    errors.push_back("VIO and PX4 local position have not passed the stable readiness gate");
    return false;
  }
  void run(const std::shared_ptr<px4_ros2::ActionHandler>& handler,
           const px4_ros2::ActionArguments& arguments, const std::function<void()>& on_completed) override
  {
    deactivate();
    _handler = handler;
    _on_completed = on_completed;
    _arguments = arguments.json();
    const auto token = _invocation.begin();
    if (!_context->command_owner() || !_context->actions_allowed() || !_context->armed()) return failure("action activated without armed command ownership");
    if (_name == "localTakeoff") _context->beginTakeoffInitialization(_arguments.at("altitude_m").get<double>());
    if (!_context->flightHealthy()) return failure("VIO/local state or mission frame invalid at action start");
    if (arguments.resuming()) return failure("automatic action resume is unsupported");
    _context->activity(_name, "running");
    handler->setActvityInfo(_name);
    if (_name == "localTakeoff") {
      if (_context->native_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN) != px4_ros2::Result::Success) {
        return failure("native PX4 Takeoff command rejected");
      }
      RCLCPP_INFO(_context->node.get_logger(), "Native PX4 Takeoff acknowledged; local handoff threshold %.2f m", kTakeoffHandoffClimbM - kTakeoffAltitudeToleranceM);
      _phase = Phase::Takeoff;
      deadline(45.0);
    } else if (_name == "localGoto") {
      const Eigen::Vector3f offset(_arguments.at("x_m").get<float>(),
                                   _arguments.at("y_m").get<float>(), _arguments.at("z_m").get<float>());
      const auto target = (_context->origin + offset).eval();
      const double dwell = _arguments.value("dwell_s", 0.5);
      std::optional<float> speed;
      if (_arguments.contains("max_horizontal_speed_m_s")) speed = _arguments.at("max_horizontal_speed_m_s").get<float>();
      const double distance = (target - _context->position()).norm();
      const double seconds = std::min(3600.0, std::max(60.0, 45.0 + 8.0 * distance / speed.value_or(1.0F) + dwell));
      runGoto(token, target, dwell, seconds, speed);
    } else if (_name == "localHold") {
      Eigen::Vector3f target = _context->position();
      if (_arguments.contains("altitude_m")) target.z() = _context->origin.z() - _arguments.at("altitude_m").get<float>();
      const double hold = _arguments.at("duration_s").get<double>();
      runGoto(token, target, hold, holdDeadlineSeconds(hold));
    } else if (_name == "orbit") {
      if (_context->vehicle_status.navState() == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT) {
        return failure("Orbit must be deactivated through a local action before reactivation");
      }
      _center = _context->position();
      const double radius = _arguments.at("radius_m").get<double>();
      const bool clockwise = _arguments.at("direction").get<std::string>() == "clockwise";
      const float signed_radius = static_cast<float>(clockwise ? radius : -radius);
      if (_context->native_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT,
                                    signed_radius, _arguments.at("speed_m_s").get<float>()) != px4_ros2::Result::Success) {
        return failure("native PX4 Orbit command rejected");
      }
      _tracker = std::make_unique<LapTracker>(_center.x(), _center.y(), radius, clockwise);
      _phase = Phase::Orbit;
      deadline(orbitDeadlineSeconds(radius, _arguments.at("laps").get<int>(), _arguments.at("speed_m_s").get<double>()));
      RCLCPP_INFO(_context->node.get_logger(), "Native Orbit started at local center (%.2f, %.2f); radius %.2f m, speed %.2f m/s", _center.x(), _center.y(), radius, _arguments.at("speed_m_s").get<double>());
    }
    // Wall ticking guarantees stalled simulator time cannot hide stale telemetry.
    if (valid(token)) _timer = _context->node.create_wall_timer(100ms, [this, token] { tick(token); });
  }
  void deactivate() override {
    _invocation.cancel();
    stopTimer();
    _handler.reset();
    _on_completed = nullptr;
    _tracker.reset();
  }
 private:
  enum class Phase { Takeoff, Goto, Orbit };
  void stopTimer() { if (_timer) { _timer->cancel(); _timer.reset(); } }
  bool valid(uint64_t token) const {
    return _invocation.valid(token) && _handler && _handler->isValid() && _context->command_owner() && _context->actions_allowed();
  }
  void deadline(double seconds) {
    _deadline = {_context->node.get_clock()->now().seconds(), monotonicSeconds(), seconds};
  }
  void failure(const std::string& reason) {
    deactivate();
    _context->fail(reason);
  }
  void complete(uint64_t token) {
    if (!valid(token) || !_invocation.finish(token)) return;
    stopTimer();
    _tracker.reset();
    const auto callback = std::move(_on_completed);
    _handler.reset();
    if (callback) callback();
  }
  void runGoto(uint64_t token, const Eigen::Vector3f& target, double dwell_s,
               double timeout_s, std::optional<float> max_speed = {}) {
    if (!valid(token)) return;
    _phase = Phase::Goto;
    deadline(timeout_s);
    _context->goto_mode->configure(target, dwell_s, max_speed);
    _handler->runMode(_context->goto_mode->id(), [] {},
      [this, token] { if (valid(token)) failure("local Goto mode failed"); });
  }
  void tick(uint64_t token) {
    if (!valid(token)) return;
    if (!_context->flightHealthy()) return failure("VIO/local state stale, invalid or reset during action");
    if (!_context->armed()) return failure("vehicle disarmed before action completion");
    if (_deadline.expired(_context->node.get_clock()->now().seconds(), monotonicSeconds())) {
      return failure(_name + " progress deadline exceeded");
    }
    if (_phase == Phase::Takeoff) {
      if (_context->local().z > _context->origin.z() - kTakeoffHandoffClimbM + kTakeoffAltitudeToleranceM) return;
      Eigen::Vector3f target = _context->origin;
      const double altitude = _arguments.at("altitude_m").get<double>();
      target.z() -= static_cast<float>(altitude);
      RCLCPP_INFO(_context->node.get_logger(), "Local EKF Takeoff handoff reached; local Goto altitude %.2f m", altitude);
      _context->initialization_goto = true;
      runGoto(token, target, 0.5, altitudeGotoDeadlineSeconds(altitude));
    } else if (_phase == Phase::Goto) {
      if (!_context->goto_mode->targetReached()) return;
      if (_name == "localTakeoff") {
        if (!_context->local().heading_good_for_control ||
            !_context->initial_frame.stable(_context->node.get_clock()->now().seconds())) return;
        _context->freezeOrigin();
      }
      complete(token);
    } else if (_phase == Phase::Orbit) {
      // ACK alone does not prove Orbit activation or acquisition.
      if (_context->vehicle_status.navState() != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT) return;
      const auto& p = _context->local();
      _tracker->update(p.x, p.y, _arguments.at("radial_tolerance_m").get<double>());
      if (_tracker->angle() >= 2.0 * std::acos(-1.0) * _arguments.at("laps").get<int>()) {
        RCLCPP_INFO(_context->node.get_logger(), "Completed %.2f rad local-position orbit angle", _tracker->angle());
        complete(token);
      }
    }
  }
  std::string _name;
  std::shared_ptr<MissionContext> _context;
  std::shared_ptr<px4_ros2::ActionHandler> _handler;
  std::function<void()> _on_completed;
  nlohmann::json _arguments;
  InvocationGuard _invocation;
  Phase _phase{Phase::Goto};
  PhaseDeadline _deadline;
  rclcpp::TimerBase::SharedPtr _timer;
  Eigen::Vector3f _center{Eigen::Vector3f::Zero()};
  std::unique_ptr<LapTracker> _tracker;
};

class LocalFailure final : public px4_ros2::ActionInterface {
 public:
  explicit LocalFailure(std::shared_ptr<MissionContext> context) : _context(std::move(context)) {}
  std::string name() const override { return "onFailure"; }
  void run(const std::shared_ptr<px4_ros2::ActionHandler>& handler,
           const px4_ros2::ActionArguments& arguments, const std::function<void()>& on_completed) override {
    // Called also for native/framework errors, not only context.fail(). Mark the
    // status failure without recursively aborting the newly installed handler.
    _context->activity("onFailure", arguments.contains("reason") ? arguments.at<std::string>("reason") : "failed");
    // Native Land uses PX4's estimator and Commander mode requirements. Stale
    // ROS local feedback must not suppress this recovery request: native
    // Takeoff can otherwise keep flying after this mission has aborted.
    // Fresh status/armed ownership remain mandatory through command_owner().
    if (_context->failure_land_requested || !_context->command_owner() || !_context->armed()) return;
    _context->failure_land_requested = true;
    RCLCPP_ERROR(_context->node.get_logger(), "Mission aborted; requesting native Land once");
    handler->runMode(px4_ros2::ModeBase::kModeIDLand, on_completed, [context = _context] {
      RCLCPP_ERROR(context->node.get_logger(), "Native recovery Land failed; PX4 Commander retains failsafe ownership");
    });
  }
 private:
  std::shared_ptr<MissionContext> _context;
};
}  // namespace

px4_ros2::MissionExecutor::Configuration localMissionConfiguration(const std::shared_ptr<MissionContext>& context)
{
  px4_ros2::MissionExecutor::Configuration configuration;
  configuration.default_actions = {"land"};
  configuration.trajectory_executor_factory = [](px4_ros2::ModeBase& mode) {
    return std::make_shared<ActionOnlyTrajectory>(mode);
  };
  for (const std::string name : {"localTakeoff", "localGoto", "localHold", "orbit"}) {
    configuration.custom_actions_factory.push_back([context, name](px4_ros2::ModeBase&) {
      return std::make_shared<LocalAction>(name, context);
    });
  }
  configuration.custom_actions_factory.push_back([context](px4_ros2::ModeBase&) {
    return std::make_shared<LocalFailure>(context);
  });
  return configuration;
}
}  // namespace uav_vio_mission
