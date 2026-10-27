"""Launch external-workspace VIO_ver1 with the ws_VIO D435i calibration."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start VIO_ver1 and normalize its IMU-frame odometry."""
    try:
        vio_estimator_prefix = Path(get_package_prefix("vio_msckf"))
    except PackageNotFoundError as error:
        raise RuntimeError(
            "vio_msckf is not discoverable; source the existing VIO_ver1 "
            "workspace before ws_VIO or run scripts/run_demo.sh"
        ) from error
    vio_estimator_executable = (
        vio_estimator_prefix / "lib" / "vio_msckf" / "run_subscribe_msckf"
    )
    if not vio_estimator_executable.is_file():
        raise RuntimeError(
            "vio_msckf is indexed but run_subscribe_msckf is missing from "
            f"{vio_estimator_prefix}; rebuild the existing VIO_ver1 workspace"
        )
    bringup_share = Path(get_package_share_directory("uav_vio_bringup"))
    config_path = bringup_share / "config" / "d435i_gazebo" / "estimator_config.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("verbosity", default_value="INFO"),
            Node(
                package="vio_msckf",
                executable="run_subscribe_msckf",
                namespace="vio_msckf",
                name="run_subscribe_msckf",
                output="screen",
                parameters=[
                    {
                        "config_path": str(config_path),
                        "verbosity": LaunchConfiguration("verbosity"),
                        "use_stereo": True,
                        "max_cameras": 2,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="uav_vio_px4_bridge",
                executable="vio_estimator_odometry_adapter",
                name="vio_estimator_odometry_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    }
                ],
            ),
        ]
    )
