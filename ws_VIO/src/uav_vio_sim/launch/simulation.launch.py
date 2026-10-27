"""Compose Gazebo, PX4 SITL, the D435i bridge, and static sensor TF."""

import fcntl
import shlex
import shutil
import socket
import subprocess
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_GZ_WORLD = "kthspacelab"
DEFAULT_GZ_VERBOSE = "1"
DEFAULT_GZ_PARTITION = "uav_vio"
DEFAULT_GZ_IP = "127.0.0.1"
DEFAULT_ROS_DOMAIN_ID = "42"
DEFAULT_ROS_LOCALHOST_ONLY = "1"
AGENT_EXECUTABLE_NAMES = {"MicroXRCEAgent", "micro-xrce-dds-agent"}
AGENT_LOCK_FILES = {}

STATIC_TRANSFORMS = (
    (
        "base_to_d435",
        "0.08",
        "0",
        "-0.10",
        "0",
        "1.57079632679",
        "0",
        "base_link",
        "d435_link",
    ),
    (
        "d435_left_frame",
        "0",
        "0.025",
        "0",
        "0",
        "0",
        "0",
        "d435_link",
        "d435_left_frame",
    ),
    (
        "d435_left_optical",
        "0",
        "0",
        "0",
        "-1.57079632679",
        "0",
        "-1.57079632679",
        "d435_left_frame",
        "d435_left_optical_frame",
    ),
    (
        "d435_right_frame",
        "0",
        "-0.025",
        "0",
        "0",
        "0",
        "0",
        "d435_link",
        "d435_right_frame",
    ),
    (
        "d435_right_optical",
        "0",
        "0",
        "0",
        "-1.57079632679",
        "0",
        "-1.57079632679",
        "d435_right_frame",
        "d435_right_optical_frame",
    ),
    (
        "d435_color_frame",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "d435_link",
        "d435_color_frame",
    ),
    (
        "d435_color_optical",
        "0",
        "0",
        "0",
        "-1.57079632679",
        "0",
        "-1.57079632679",
        "d435_color_frame",
        "d435_color_optical_frame",
    ),
    (
        "d435_depth_frame",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "d435_link",
        "d435_depth_frame",
    ),
    (
        "d435_depth_optical",
        "0",
        "0",
        "0",
        "-1.57079632679",
        "0",
        "-1.57079632679",
        "d435_depth_frame",
        "d435_depth_optical_frame",
    ),
    (
        "d435_imu",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "d435_link",
        "d435_imu_frame",
    ),
)


def _running_agent_ports() -> list[tuple[str, str | None]]:
    process_list = subprocess.run(
        ["ps", "-eo", "args="],
        check=True,
        capture_output=True,
        text=True,
    )
    agents = []
    for command_line in process_list.stdout.splitlines():
        try:
            arguments = shlex.split(command_line)
        except ValueError:
            continue
        if not arguments or Path(arguments[0]).name not in AGENT_EXECUTABLE_NAMES:
            continue

        port = None
        for index, argument in enumerate(arguments):
            if argument in ("-p", "--port") and index + 1 < len(arguments):
                port = arguments[index + 1]
                break
            if argument.startswith("--port="):
                port = argument.split("=", maxsplit=1)[1]
                break
        agents.append((command_line, port))
    return agents


def _acquire_agent_lock(port: str) -> None:
    lock_path = Path(tempfile.gettempdir()) / f"uav_vio_xrce_{port}.lock"
    lock_file = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_file.close()
        raise RuntimeError(
            f"Another ws_VIO launch is already managing XRCE-DDS UDP port {port}"
        ) from exc
    AGENT_LOCK_FILES[port] = lock_file


def _udp_port_is_bound(port: int) -> bool:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
    except OSError:
        return True
    finally:
        probe.close()
    return False


def _agent_action(context, *args, **kwargs):
    del args, kwargs
    mode = LaunchConfiguration("agent_mode").perform(context)
    port = LaunchConfiguration("micro_xrce_port").perform(context)
    try:
        port_number = int(port)
    except ValueError as exc:
        raise ValueError("micro_xrce_port must be an integer") from exc
    if not 1 <= port_number <= 65535:
        raise ValueError("micro_xrce_port must be between 1 and 65535")
    if mode not in ("auto", "existing", "start"):
        raise RuntimeError("agent_mode must be auto, existing, or start")

    _acquire_agent_lock(port)

    running_agents = _running_agent_ports()
    matching_agents = [agent for agent in running_agents if agent[1] == port]
    if matching_agents:
        return []
    if running_agents:
        details = "; ".join(
            f"{command_line} (port={running_port or 'unknown'})"
            for command_line, running_port in running_agents
        )
        raise RuntimeError(
            f"XRCE-DDS Agent already runs on a different or unknown port: {details}"
        )
    if _udp_port_is_bound(port_number):
        raise RuntimeError(
            f"UDP port {port} is already bound by an unrecognized process; "
            "refusing to start a duplicate XRCE-DDS Agent"
        )

    if mode == "existing":
        raise RuntimeError(
            f"agent_mode=existing but no XRCE-DDS Agent uses UDP port {port}"
        )
    executable = shutil.which("MicroXRCEAgent") or shutil.which(
        "micro-xrce-dds-agent"
    )
    if executable is None:
        raise RuntimeError("Micro XRCE-DDS Agent executable was not found")
    agent_process = ExecuteProcess(
        cmd=[executable, "udp4", "-p", port],
        output="screen",
    )
    shutdown_on_agent_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=agent_process,
            on_exit=[
                LogInfo(
                    msg=(
                        "Micro XRCE-DDS Agent exited; shutting down simulation."
                    )
                ),
                EmitEvent(event=Shutdown(reason="Micro XRCE-DDS Agent exited")),
            ],
        )
    )
    return [shutdown_on_agent_exit, agent_process]


def _validate_dds_isolation(context, *args, **kwargs):
    del args, kwargs
    domain_value = LaunchConfiguration("ros_domain_id").perform(context)
    try:
        domain_id = int(domain_value)
    except ValueError as exc:
        raise ValueError("ros_domain_id must be an integer") from exc
    if not 0 <= domain_id <= 232:
        raise ValueError("ros_domain_id must be between 0 and 232")

    localhost_only = LaunchConfiguration("ros_localhost_only").perform(context)
    if localhost_only not in ("0", "1"):
        raise ValueError("ros_localhost_only must be 0 or 1")
    return []


def _static_transform_nodes():
    nodes = []
    for name, x, y, z, roll, pitch, yaw, parent, child in STATIC_TRANSFORMS:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{name}_tf",
                arguments=[
                    "--x",
                    x,
                    "--y",
                    y,
                    "--z",
                    z,
                    "--roll",
                    roll,
                    "--pitch",
                    pitch,
                    "--yaw",
                    yaw,
                    "--frame-id",
                    parent,
                    "--child-frame-id",
                    child,
                ],
                parameters=[{"use_sim_time": True}],
            )
        )
    return nodes


def generate_launch_description():
    """Create the low-level simulation launch description."""
    pkg_share_dir = Path(get_package_share_directory("uav_vio_sim"))

    arguments = [
        DeclareLaunchArgument(
            "px4_dir",
            default_value=EnvironmentVariable("PX4_DIR", default_value=""),
            description="Path to PX4-Autopilot checkout (defaults to env PX4_DIR)",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=DEFAULT_GZ_WORLD,
            description="Gazebo world name, SDF basename, or absolute SDF path",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="If true: start the Gazebo GUI after its server",
        ),
        DeclareLaunchArgument(
            "gz_verbose",
            default_value=DEFAULT_GZ_VERBOSE,
            description="Gazebo verbosity level for `gz sim --verbose`",
        ),
        DeclareLaunchArgument(
            "gz_partition",
            default_value=DEFAULT_GZ_PARTITION,
            description="Gazebo partition shared by Gazebo, PX4, and ros_gz_bridge",
        ),
        DeclareLaunchArgument(
            "gz_ip",
            default_value=DEFAULT_GZ_IP,
            description="Gazebo IP shared by Gazebo, PX4, and ros_gz_bridge",
        ),
        DeclareLaunchArgument(
            "vehicle_id",
            default_value="0",
            description="PX4 instance ID (-i)",
        ),
        DeclareLaunchArgument(
            "model_pose",
            default_value="0,0",
            description=(
                "Pose for PX4_GZ_MODEL_POSE: x,y or x,y,z or "
                "x,y,z,roll,pitch,yaw"
            ),
        ),
        DeclareLaunchArgument(
            "px4_sys_autostart",
            default_value="4001",
            description="PX4 autostart ID (PX4_SYS_AUTOSTART)",
        ),
        DeclareLaunchArgument(
            "px4_sim_model",
            default_value="gz_x500_d435i",
            description="PX4 Gazebo model name (PX4_SIM_MODEL)",
        ),
        DeclareLaunchArgument(
            "px4_parameter_profile",
            default_value="gnss_vio",
            description="Startup PX4 profile: gnss_vio or vio_only",
        ),
        DeclareLaunchArgument(
            "agent_mode",
            default_value="auto",
            description="XRCE-DDS Agent policy: auto, existing, or start",
        ),
        DeclareLaunchArgument(
            "micro_xrce_port",
            default_value="8888",
            description="UDP port used by the Micro XRCE-DDS Agent",
        ),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value=EnvironmentVariable(
                "ROS_DOMAIN_ID", default_value=DEFAULT_ROS_DOMAIN_ID
            ),
            description="Isolated ROS/PX4 DDS domain (0-232)",
        ),
        DeclareLaunchArgument(
            "ros_localhost_only",
            default_value=EnvironmentVariable(
                "ROS_LOCALHOST_ONLY",
                default_value=DEFAULT_ROS_LOCALHOST_ONLY,
            ),
            description="Restrict ROS/PX4 DDS discovery to localhost: 0 or 1",
        ),
        DeclareLaunchArgument(
            "world_ready_timeout",
            default_value="30.0",
            description="Seconds to wait for the Gazebo scene/info service",
        ),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share_dir / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in (
                "px4_dir",
                "world",
                "gui",
                "gz_verbose",
                "gz_partition",
                "gz_ip",
            )
        }.items(),
    )
    px4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share_dir / "launch" / "px4_spawn.launch.py")
        ),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in (
                "px4_dir",
                "world",
                "px4_sys_autostart",
                "px4_sim_model",
                "px4_parameter_profile",
                "vehicle_id",
                "model_pose",
                "gz_partition",
                "gz_ip",
                "world_ready_timeout",
                "micro_xrce_port",
                "ros_domain_id",
                "ros_localhost_only",
            )
        }.items(),
    )
    d435i_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="d435i_gz_bridge",
        parameters=[
            {
                "config_file": str(pkg_share_dir / "config" / "d435i_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            *arguments,
            SetEnvironmentVariable(
                "GZ_PARTITION", LaunchConfiguration("gz_partition")
            ),
            SetEnvironmentVariable("GZ_IP", LaunchConfiguration("gz_ip")),
            OpaqueFunction(function=_validate_dds_isolation),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            SetEnvironmentVariable(
                "ROS_LOCALHOST_ONLY",
                LaunchConfiguration("ros_localhost_only"),
            ),
            OpaqueFunction(function=_agent_action),
            gz_sim,
            px4_spawn,
            d435i_bridge,
            *_static_transform_nodes(),
        ]
    )
