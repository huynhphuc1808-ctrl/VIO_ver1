from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="vio_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="vio_enable", default_value="true", description="enable VIO_ver1 node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="use ROS /clock instead of the host wall clock",
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    )
]

def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("vio_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(
                            get_package_share_directory("vio_msckf"),
                            "config",config,"estimator_config.yaml"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting VIO_ver1".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting VIO_ver1".format(
                        config_path)
                    )
            ]
    node1 = Node(
        package="vio_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("vio_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"config_path": config_path},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("vio_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
            ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return [node1, node2]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
