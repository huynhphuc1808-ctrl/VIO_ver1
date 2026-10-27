#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <std_msgs/msg/string.hpp>
#include "uav_vio_mission/mission_guards.hpp"

namespace uav_vio_mission {

double monotonicSeconds();

struct PhaseDeadline {
  double clock_start{}, wall_start{}, seconds{};
  bool expired(double clock_now, double wall_now) const {
    return clock_now < clock_start || clock_now - clock_start >= seconds ||
           wall_now - wall_start >= 4.0 * seconds + 10.0;
  }
};

class LocalGotoMode;
struct MissionContext {
  MissionContext(rclcpp::Node& node, double gate_stable_seconds);
  bool localFresh() const;
  bool statusFresh() const;
  bool vehicleFresh() const;
  bool landed() const;
  bool armed() const;
  bool gateReady();
  bool flightHealthy() const;
  bool frameValid() const;
  void captureOrigin();
  void beginTakeoffInitialization(double altitude_m);
  void refreshInitialization();
  void freezeOrigin();
  FrameEpoch epoch() const;
  Eigen::Vector3f position() const;
  const px4_msgs::msg::VehicleLocalPosition& local() const { return local_position.last(); }

  rclcpp::Node& node;
  px4_ros2::Context px4_context;
  px4_ros2::OdometryLocalPosition local_position;
  px4_ros2::VehicleStatus vehicle_status;
  px4_ros2::LandDetected land_detected;
  StableReadiness readiness;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vio_subscription;
  std::optional<double> status_received, land_received;
  std::optional<FrameEpoch> origin_epoch;
  Eigen::Vector3f origin{Eigen::Vector3f::Zero()};
  std::shared_ptr<LocalGotoMode> goto_mode;
  InitialFrameState initial_frame;
  bool initializing{false}, frame_frozen{false}, initialization_invalid{false}, initialization_goto{false};
  double initialization_altitude_m{2.0};
  // Bound by the runtime after constructing its single upstream executor.
  std::function<bool()> command_owner;
  std::function<bool()> actions_allowed;
  std::function<px4_ros2::Result(uint32_t, float, float)> native_command;
  std::function<void(const std::string&)> fail;
  std::function<void(const std::string&, const std::string&)> activity;
  bool mission_valid{false};
  bool failure_land_requested{false};
};

class LocalGotoMode : public px4_ros2::ModeBase {
 public:
  LocalGotoMode(rclcpp::Node& node, const std::shared_ptr<MissionContext>& context);
  void configure(const Eigen::Vector3f& target, double dwell_s,
                 std::optional<float> max_horizontal_speed = {});
  bool targetReached() const { return _completed; }
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;
 private:
  std::weak_ptr<MissionContext> _context;
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_requirements;
  Eigen::Vector3f _target{Eigen::Vector3f::Zero()};
  std::optional<float> _max_horizontal_speed;
  std::optional<double> _reached_since;
  double _dwell_s{0.5};
  bool _configured{false}, _completed{false};
};
}  // namespace uav_vio_mission
