#pragma once

#include <string>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include "uav_vio_mission/mission_logic.hpp"

namespace uav_vio_mission {

// Flight-independent contract shared by the runtime, CLI and planner tests.
// Stock global navigation and library internal:* arguments are deliberately rejected.
struct MissionDefinition {
  nlohmann::json json;
  px4_ros2::Mission mission;
  std::string revision;
};
MissionDefinition validateMission(const nlohmann::json& json);
MissionDefinition loadMission(const std::string& filename);
nlohmann::json circleMission(const MissionParameters& parameters);

}  // namespace uav_vio_mission
