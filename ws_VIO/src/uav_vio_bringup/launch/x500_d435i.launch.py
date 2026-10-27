"""Compose continuous external vision for PX4's onboard Mission navigator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the complete x500 D435i bringup."""
    sim_share = Path(get_package_share_directory("uav_vio_sim"))
    bridge_share = Path(get_package_share_directory("uav_vio_px4_bridge"))
    bringup_share = Path(get_package_share_directory("uav_vio_bringup"))

    arguments = [
        DeclareLaunchArgument(
            "px4_dir",
            default_value=EnvironmentVariable("PX4_DIR", default_value=""),
        ),
        DeclareLaunchArgument("world", default_value="kthspacelab"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("vehicle_id", default_value="0"),
        DeclareLaunchArgument(
            "model_pose",
            default_value="0,0",
            description="Pose string: x,y or x,y,z or x,y,z,roll,pitch,yaw",
        ),
        DeclareLaunchArgument("px4_sys_autostart", default_value="4001"),
        DeclareLaunchArgument("px4_sim_model", default_value="gz_x500_d435i"),
        DeclareLaunchArgument("px4_parameter_profile", default_value="gnss_vio"),
        DeclareLaunchArgument(
            "external_odom_topic",
            default_value="/vio/odometry",
        ),
        DeclareLaunchArgument("vio_estimator", default_value="true"),
        DeclareLaunchArgument("agent_mode", default_value="auto"),
        DeclareLaunchArgument("micro_xrce_port", default_value="8888"),
        DeclareLaunchArgument("gz_verbose", default_value="1"),
        DeclareLaunchArgument("gz_partition", default_value="uav_vio"),
        DeclareLaunchArgument("gz_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="42"),
        ),
        DeclareLaunchArgument(
            "ros_localhost_only",
            default_value=EnvironmentVariable(
                "ROS_LOCALHOST_ONLY", default_value="1"
            ),
        ),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / "launch" / "simulation.launch.py")
        ),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in (
                "px4_dir",
                "world",
                "gui",
                "vehicle_id",
                "model_pose",
                "px4_sys_autostart",
                "px4_sim_model",
                "px4_parameter_profile",
                "agent_mode",
                "micro_xrce_port",
                "gz_verbose",
                "gz_partition",
                "gz_ip",
                "ros_domain_id",
                "ros_localhost_only",
            )
        }.items(),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bridge_share / "launch" / "px4_bridge.launch.py")
        ),
        launch_arguments={
            "input_topic": LaunchConfiguration("external_odom_topic"),
            "model_pose": LaunchConfiguration("model_pose"),
            "viz_enabled": LaunchConfiguration("rviz"),
            "use_sim_time": "true",
        }.items(),
    )
    vio_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "vio_estimator.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("vio_estimator")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="d435i_rviz",
        arguments=["-d", str(bringup_share / "rviz" / "d435i.rviz")],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription(
        [
            *arguments,
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            SetEnvironmentVariable(
                "ROS_LOCALHOST_ONLY",
                LaunchConfiguration("ros_localhost_only"),
            ),
            simulation,
            vio_estimator,
            bridge,
            rviz,
        ]
    )
