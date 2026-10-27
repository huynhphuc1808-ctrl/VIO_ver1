#include "uav_vio_mission/mission_context.hpp"

#include <chrono>
#include <cmath>

namespace uav_vio_mission {
double monotonicSeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

MissionContext::MissionContext(rclcpp::Node& n, double stable_seconds)
    : node(n), px4_context(n), local_position(px4_context), vehicle_status(px4_context),
      land_detected(px4_context), readiness(stable_seconds)
{
  local_position.onUpdate([this](const px4_msgs::msg::VehicleLocalPosition& p) {
    // API 1 Subscription::last() is unavailable when lastTime() is zero.
    // Telemetry can arrive before the first /clock sample on a new ROS node.
    const bool valid = local_position.lastTime().nanoseconds() > 0 &&
        p.xy_valid && p.z_valid && p.v_xy_valid && p.v_z_valid &&
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(p.vx) && std::isfinite(p.vy) && std::isfinite(p.vz);
    readiness.local(monotonicSeconds(), valid);
    if (initializing && valid) refreshInitialization();
  });
  vehicle_status.onUpdate([this](const px4_msgs::msg::VehicleStatus&) {
    if (vehicle_status.lastTime().nanoseconds() > 0) status_received = monotonicSeconds();
    else status_received.reset();
  });
  land_detected.onUpdate([this](const px4_msgs::msg::VehicleLandDetected&) {
    if (land_detected.lastTime().nanoseconds() > 0) land_received = monotonicSeconds();
    else land_received.reset();
  });
  vio_subscription = n.create_subscription<std_msgs::msg::String>(
      "/vio/status", rclcpp::QoS(10), [this](const std_msgs::msg::String& msg) {
        readiness.vio(monotonicSeconds(), msg.data == "VALID_ODOMETRY");
      });
}
bool MissionContext::localFresh() const {
  return local_position.lastTime().nanoseconds() > 0 && readiness.localFresh(monotonicSeconds());
}
bool MissionContext::statusFresh() const {
  return status_received && vehicle_status.lastTime().nanoseconds() > 0 &&
      monotonicSeconds() - *status_received < 2.0;
}
bool MissionContext::vehicleFresh() const
{
  // LandDetected is a low-rate event/heartbeat; at slow SITL real-time factors
  // one simulator second spans multiple wall seconds. It is not a command lease.
  return statusFresh() && land_received && monotonicSeconds() - *land_received < 5.0;
}
bool MissionContext::landed() const {
  return land_received && land_detected.lastTime().nanoseconds() > 0 && land_detected.landed();
}
bool MissionContext::armed() const {
  return status_received && vehicle_status.lastTime().nanoseconds() > 0 && vehicle_status.armed();
}
bool MissionContext::gateReady()
{
  const double now = monotonicSeconds();
  readiness.observe(now);
  return node.get_clock()->now().nanoseconds() > 0 && mission_valid &&
         readiness.ready(now) && vehicleFresh();
}
bool MissionContext::flightHealthy() const
{
  return node.get_clock()->now().nanoseconds() > 0 && readiness.fresh(monotonicSeconds()) &&
         (initializing ? !initialization_invalid : frameValid());
}
FrameEpoch MissionContext::epoch() const
{
  const auto& p = local();
  return {p.xy_reset_counter, p.z_reset_counter, p.vxy_reset_counter, p.vz_reset_counter,
          p.heading_reset_counter, p.ref_timestamp};
}
Eigen::Vector3f MissionContext::position() const
{
  const auto& p = local();
  return {p.x, p.y, p.z};
}
bool MissionContext::frameValid() const
{
  return frame_frozen && origin_epoch && localFresh() && *origin_epoch == epoch();
}
void MissionContext::captureOrigin()
{
  if (!localFresh()) throw std::runtime_error("cannot freeze mission origin without fresh local state");
  origin = position();
  origin_epoch = epoch();
  frame_frozen = false;
  initializing = false;
  initialization_invalid = false;
  initialization_goto = false;
  initial_frame.capture(*origin_epoch, origin.z(), node.get_clock()->now().seconds());
}
void MissionContext::beginTakeoffInitialization(double altitude_m)
{
  initializing = true;
  initialization_altitude_m = altitude_m;
  refreshInitialization();
}
void MissionContext::refreshInitialization()
{
  if (!initializing || initialization_invalid || !localFresh()) return;
  const auto update = initial_frame.observe(epoch(), local().delta_z, node.get_clock()->now().seconds());
  if (update == InitialFrameState::Update::Invalid) {
    initialization_invalid = true;
    if (fail) fail("unexplained local frame reset during Takeoff initialization");
    return;
  }
  if (update != InitialFrameState::Update::Retarget) return;
  origin = position();
  origin.z() = static_cast<float>(initial_frame.groundZ());
  origin_epoch = epoch();
  // This executes inside the local subscription callback before any 30 Hz mode
  // update can publish a target in the preceding epoch. Deadline is not reset.
  if (initialization_goto && goto_mode) {
    Eigen::Vector3f target = origin;
    target.z() -= static_cast<float>(initialization_altitude_m);
    goto_mode->configure(target, 0.5);
  }
  RCLCPP_INFO(node.get_logger(), "Takeoff alignment reset: rebased temporary local hold target; mission frame is still provisional");
}
void MissionContext::freezeOrigin()
{
  if (!initializing || !localFresh() || initialization_invalid || !local().heading_good_for_control ||
      !initial_frame.stable(node.get_clock()->now().seconds())) throw std::runtime_error("local frame not settled for freeze");
  origin = position();
  origin.z() = static_cast<float>(initial_frame.groundZ());
  origin_epoch = epoch();
  frame_frozen = true;
  initializing = false;
  initialization_goto = false;
  RCLCPP_INFO(node.get_logger(), "Frozen aligned mission_ned origin (%.2f, %.2f, %.2f)", origin.x(), origin.y(), origin.z());
}

LocalGotoMode::LocalGotoMode(rclcpp::Node& node, const std::shared_ptr<MissionContext>& context)
    : ModeBase(node, Settings("VIO Local Goto")), _context(context),
      _goto(std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this)),
      _local_requirements(std::make_shared<px4_ros2::OdometryLocalPosition>(*this))
{
}
void LocalGotoMode::configure(const Eigen::Vector3f& target, double dwell_s,
                              std::optional<float> max_horizontal_speed)
{
  _target = target;
  _dwell_s = dwell_s;
  _max_horizontal_speed = max_horizontal_speed;
  _configured = true;
  _completed = false;
  _reached_since.reset();
}
void LocalGotoMode::onActivate() { _reached_since.reset(); _completed = false; }
void LocalGotoMode::onDeactivate() { _reached_since.reset(); }
void LocalGotoMode::checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter)
{
  const auto context = _context.lock();
  if (!context || (!context->armed() ? !context->gateReady() : !context->flightHealthy())) {
    reporter.armingCheckFailureExt(px4_ros2::events::ID("vio_local_mode_not_ready"),
                                  px4_ros2::events::Log::Error, "VIO local mode not ready");
  }
}
void LocalGotoMode::updateSetpoint(float)
{
  const auto context = _context.lock();
  if (!context || !_configured || !context->command_owner || !context->command_owner() ||
      !context->actions_allowed || !context->actions_allowed()) return;
  if (!context->flightHealthy()) {
    _completed = true;
    completed(px4_ros2::Result::ModeFailureOther);
    return;
  }
  _goto->update(_target, {}, _max_horizontal_speed);
  const auto& p = context->local();
  const bool reached = (context->position() - _target).norm() < 0.5F &&
                      Eigen::Vector3f(p.vx, p.vy, p.vz).norm() < 0.3F;
  const double now = node().get_clock()->now().seconds();
  if (!reached || (_reached_since && now < *_reached_since)) _reached_since.reset();
  if (reached && !_reached_since) _reached_since = now;
  if (_reached_since && now - *_reached_since >= _dwell_s) {
    _completed = true;
    // Action completion is consumed by the ActionInterface timer. ModeBase
    // success is one-shot until mode reactivation, so repeated local actions
    // intentionally retain this mode and keep supplying a safe hold setpoint.
  }
}
}  // namespace uav_vio_mission
