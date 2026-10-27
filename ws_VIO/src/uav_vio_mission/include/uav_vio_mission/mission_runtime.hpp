#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace uav_vio_mission {
// Owns one PX4 MissionExecutor backend. Mission sequencing belongs to upstream;
// this object only handles loading, prearm bootstrap and observing termination.
class MissionRuntime {
 public:
  explicit MissionRuntime(std::shared_ptr<rclcpp::Node> node);
  ~MissionRuntime();
  void start();
 private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};
}  // namespace uav_vio_mission
