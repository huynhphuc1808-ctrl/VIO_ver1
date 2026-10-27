#include "uav_vio_mission/mission_definition.hpp"
#include "uav_vio_mission/mission_runtime.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
  // Planner/offline validation does not initialize ROS or create command endpoints.
  if (argc >= 2 && std::string(argv[1]) == "--validate-mission") {
    if (argc != 3) { std::cerr << "usage: vio_mission --validate-mission FILE\n"; return 2; }
    try {
      const auto definition = uav_vio_mission::loadMission(argv[2]);
      std::cout << "valid revision=" << definition.revision << " items=" << definition.mission.items().size() << '\n';
      return 0;
    } catch (const std::exception& error) { std::cerr << "invalid mission: " << error.what() << '\n'; return 2; }
  }
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("uav_vio_mission");
    uav_vio_mission::MissionRuntime runtime(node);
    runtime.start();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    std::cerr << "VIO mission startup/runtime error: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
