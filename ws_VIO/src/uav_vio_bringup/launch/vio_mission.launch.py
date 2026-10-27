"""JSON MissionExecutor runtime on the VIO-only PX4 SITL stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("uav_vio_bringup"))
    arguments = [
        DeclareLaunchArgument("px4_dir", default_value=EnvironmentVariable("PX4_DIR", default_value="")),
        DeclareLaunchArgument("world", default_value="kthspacelab"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("vehicle_id", default_value="0"),
        DeclareLaunchArgument("model_pose", default_value="0,0"),
        DeclareLaunchArgument("agent_mode", default_value="auto"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("mission_file", default_value="", description="JSON v1 mission; empty builds circle from parameters"),
        DeclareLaunchArgument("radius_m", default_value="2.0"),
        DeclareLaunchArgument("altitude_m", default_value="2.0"),
        DeclareLaunchArgument("speed_m_s", default_value="1.0"),
        DeclareLaunchArgument("direction", default_value="clockwise"),
        DeclareLaunchArgument("laps", default_value="1"),
        DeclareLaunchArgument("hold_seconds", default_value="2.0"),
        DeclareLaunchArgument("radial_tolerance_m", default_value="0.5"),
        DeclareLaunchArgument("gate_stable_seconds", default_value="2.0"),
    ]
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "x500_d435i.launch.py")),
        launch_arguments=({
            key: LaunchConfiguration(key)
            for key in ("px4_dir", "world", "gui", "rviz", "vehicle_id", "model_pose", "agent_mode")
        } | {"px4_parameter_profile": "vio_only", "vio_estimator": "true"}).items(),
    )
    parameters = {
        "use_sim_time": True,
        "mission_file": ParameterValue(LaunchConfiguration("mission_file"), value_type=str),
        "direction": ParameterValue(LaunchConfiguration("direction"), value_type=str),
        "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
        "laps": ParameterValue(LaunchConfiguration("laps"), value_type=int),
    }
    parameters.update({
        key: ParameterValue(LaunchConfiguration(key), value_type=float)
        for key in ("radius_m", "altitude_m", "speed_m_s", "hold_seconds",
                    "radial_tolerance_m", "gate_stable_seconds")
    })
    mission = Node(package="uav_vio_mission", executable="vio_mission",
                   name="uav_vio_mission", parameters=[parameters], output="screen")
    return LaunchDescription([*arguments, base, mission])
