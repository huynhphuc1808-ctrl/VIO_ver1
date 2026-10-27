#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include "uav_vio_mission/mission_context.hpp"

namespace uav_vio_mission {
px4_ros2::MissionExecutor::Configuration localMissionConfiguration(
    const std::shared_ptr<MissionContext>& context);
}  // namespace uav_vio_mission
