"""Public circle preset of the single JSON mission runtime.

Existing circle arguments are declared by the included launch description.
An empty mission_file generates the circle definition from those arguments.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory("uav_vio_bringup"))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "vio_mission.launch.py")))
    ])
