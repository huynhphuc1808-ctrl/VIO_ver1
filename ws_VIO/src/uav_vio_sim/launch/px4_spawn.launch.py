import math
import shlex
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from uav_vio_sim.launch_support import (
    _gz_shell_setup,
    _read_world_name,
    _resolve_world_path,
)


PX4_GZ_NO_FOLLOW = "1"
DEFAULT_GZ_WORLD = "lawn"
DEFAULT_GZ_PARTITION = "uav_vio"
DEFAULT_GZ_IP = "127.0.0.1"
DEFAULT_ROS_DOMAIN_ID = "42"
DEFAULT_ROS_LOCALHOST_ONLY = "1"


def _normalize_model_pose(pose: str) -> str:
    values = [value.strip() for value in str(pose).split(",") if value.strip()]
    if len(values) == 2:
        x, y = values
        z, roll, pitch, yaw = "0", "0", "0", "0"
    elif len(values) == 3:
        x, y, z = values
        roll, pitch, yaw = "0", "0", "0"
    elif len(values) == 6:
        x, y, z, roll, pitch, yaw = values
    else:
        raise ValueError(
            "model_pose must be 'x,y', 'x,y,z' or "
            f"'x,y,z,roll,pitch,yaw': {pose!r}"
        )
    return f"{x},{y},{z},{roll},{pitch},{yaw}"


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser()
    world = LaunchConfiguration("world").perform(context)
    px4_sys_autostart = LaunchConfiguration("px4_sys_autostart").perform(context)
    px4_sim_model = LaunchConfiguration("px4_sim_model").perform(context)
    vehicle_id = LaunchConfiguration("vehicle_id").perform(context)
    model_pose = _normalize_model_pose(
        LaunchConfiguration("model_pose").perform(context)
    )
    gz_partition = LaunchConfiguration("gz_partition").perform(context)
    gz_ip = LaunchConfiguration("gz_ip").perform(context)
    world_ready_timeout = float(
        LaunchConfiguration("world_ready_timeout").perform(context)
    )
    micro_xrce_port = LaunchConfiguration("micro_xrce_port").perform(context)
    ros_domain_value = LaunchConfiguration("ros_domain_id").perform(context)
    ros_localhost_only = LaunchConfiguration("ros_localhost_only").perform(
        context
    )
    if not math.isfinite(world_ready_timeout) or world_ready_timeout <= 0:
        raise ValueError("world_ready_timeout must be finite and greater than zero")
    try:
        micro_xrce_port_number = int(micro_xrce_port)
    except ValueError as exc:
        raise ValueError("micro_xrce_port must be an integer") from exc
    if not 1 <= micro_xrce_port_number <= 65535:
        raise ValueError("micro_xrce_port must be between 1 and 65535")
    try:
        ros_domain_id = int(ros_domain_value)
    except ValueError as exc:
        raise ValueError("ros_domain_id must be an integer") from exc
    if not 0 <= ros_domain_id <= 232:
        raise ValueError("ros_domain_id must be between 0 and 232")
    if ros_localhost_only not in ("0", "1"):
        raise ValueError("ros_localhost_only must be 0 or 1")

    if not px4_dir.is_dir():
        raise RuntimeError(
            "px4_dir is empty or invalid. Set env PX4_DIR or pass "
            "px4_dir:=/path/to/PX4-Autopilot"
        )

    px4_binary = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"
    gz_env_sh = px4_dir / "build" / "px4_sitl_default" / "rootfs" / "gz_env.sh"
    px4_plugins_dir = (
        px4_dir
        / "build"
        / "px4_sitl_default"
        / "src"
        / "modules"
        / "simulation"
        / "gz_plugins"
    )
    px4_server_config = (
        px4_dir / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"
    )
    for required in (
        px4_binary,
        gz_env_sh,
        px4_plugins_dir,
        px4_server_config,
    ):
        if not required.exists():
            raise RuntimeError(f"Required PX4 Gazebo artifact not found: {required}")

    pkg_share_dir = Path(get_package_share_directory("uav_vio_sim"))
    pkg_prefix = Path(get_package_prefix("uav_vio_sim"))
    pkg_models_dir = pkg_share_dir / "models"
    pkg_worlds_dir = pkg_share_dir / "worlds"
    profile_name = LaunchConfiguration("px4_parameter_profile").perform(context)
    if profile_name not in ("gnss_vio", "vio_only"):
        raise ValueError("px4_parameter_profile must be gnss_vio or vio_only")
    navigation_profile = pkg_share_dir / "config" / f"{profile_name}.params"
    parameter_env = {}
    for line in navigation_profile.read_text().splitlines():
        fields = shlex.split(line, comments=True)
        if not fields:
            continue
        if len(fields) != 2 or not fields[0].replace("_", "").isalnum():
            raise ValueError(f"Invalid PX4 parameter profile line: {line}")
        if not math.isfinite(float(fields[1])):
            raise ValueError(f"Non-finite PX4 parameter: {line}")
        parameter_env[f"PX4_PARAM_{fields[0]}"] = fields[1]
    source_world = _resolve_world_path(pkg_worlds_dir=pkg_worlds_dir, world=world)
    world_name = _read_world_name(source_world)
    gz_shell_setup = _gz_shell_setup(
        px4_dir=str(px4_dir),
        gz_env_sh=str(gz_env_sh),
        pkg_models_dir=str(pkg_models_dir),
        pkg_worlds_dir=str(pkg_worlds_dir),
        px4_plugins_dir=str(px4_plugins_dir),
        px4_server_config=str(px4_server_config),
        gz_partition=gz_partition,
        gz_ip=gz_ip,
    )

    px4_cmd = (
        f"{gz_shell_setup}"
        " && env "
        f"PX4_GZ_NO_FOLLOW={shlex.quote(PX4_GZ_NO_FOLLOW)} "
        f"PX4_GZ_WORLD={shlex.quote(world_name)} "
        "PX4_GZ_STANDALONE=1 "
        f"PX4_GZ_MODELS={shlex.quote(str(pkg_models_dir))} "
        f"PX4_UXRCE_DDS_PORT={shlex.quote(micro_xrce_port)} "
        f"PX4_PARAM_UXRCE_DDS_PTCFG={shlex.quote(ros_localhost_only)} "
        f"PX4_SYS_AUTOSTART={shlex.quote(px4_sys_autostart)} "
        f"PX4_SIM_MODEL={shlex.quote(px4_sim_model)} "
        f"PX4_GZ_MODEL_POSE={shlex.quote(model_pose)} "
        f"{shlex.quote(str(px4_binary))} -i {shlex.quote(vehicle_id)}"
    )
    px4_process = ExecuteProcess(
        cmd=["bash", "-c", px4_cmd],
        additional_env=parameter_env,
        output="screen",
    )
    configure_px4 = ExecuteProcess(
        cmd=[
            "bash",
            str(pkg_prefix / "lib" / "uav_vio_sim" / "configure_px4_sitl.sh"),
            str(px4_dir),
            vehicle_id,
            str(navigation_profile),
        ],
        output="screen",
    )

    world_service = f"/world/{world_name}/scene/info"
    probe_attempts = max(1, math.ceil(world_ready_timeout / 0.2))
    world_probe_cmd = (
        f"{gz_shell_setup}"
        f" && echo {shlex.quote(f'Waiting for Gazebo service {world_service}')}"
        f" && attempts={probe_attempts}"
        " && while [ \"$attempts\" -gt 0 ]; do "
        f"if gz service -i --service {shlex.quote(world_service)} 2>&1 "
        "| grep -q 'Service providers'; then exit 0; fi; "
        "attempts=$((attempts - 1)); sleep 0.2; done; "
        f"echo {shlex.quote(f'Timed out waiting for Gazebo service {world_service}')} "
        ">&2; exit 1"
    )
    world_probe = ExecuteProcess(
        cmd=["bash", "-c", world_probe_cmd],
        output="screen",
    )

    def _on_world_probe_exit(event, launch_context):
        del launch_context
        if event.returncode == 0:
            return [
                LogInfo(msg=f"Gazebo world '{world_name}' is ready; starting PX4."),
                px4_process,
                configure_px4,
            ]
        return [
            LogInfo(msg=f"Gazebo world '{world_name}' did not become ready."),
            EmitEvent(event=Shutdown(reason="Gazebo world readiness failed")),
        ]

    def _on_configure_px4_exit(event, launch_context):
        del launch_context
        if event.returncode == 0:
            return [LogInfo(msg="PX4 GNSS + VIO startup parameters verified.")]
        return [
            LogInfo(msg="PX4 GNSS + VIO startup parameter verification failed."),
            EmitEvent(event=Shutdown(reason="PX4 configuration failed")),
        ]

    start_px4_after_world = RegisterEventHandler(
        OnProcessExit(
            target_action=world_probe,
            on_exit=_on_world_probe_exit,
        )
    )
    shutdown_on_configure_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_px4,
            on_exit=_on_configure_px4_exit,
        )
    )
    shutdown_on_px4_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=px4_process,
            on_exit=[
                LogInfo(msg="PX4 exited; shutting down simulation."),
                EmitEvent(event=Shutdown(reason="PX4 exited")),
            ],
        )
    )
    return [
        start_px4_after_world,
        shutdown_on_configure_failure,
        shutdown_on_px4_exit,
        world_probe,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_dir",
                default_value=EnvironmentVariable("PX4_DIR", default_value=""),
                description="Path to PX4-Autopilot checkout (defaults to env PX4_DIR)",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=DEFAULT_GZ_WORLD,
                description="Gazebo world name used by PX4 (PX4_GZ_WORLD)",
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
                "gz_partition",
                default_value=DEFAULT_GZ_PARTITION,
                description="Gazebo partition name (GZ_PARTITION)",
            ),
            DeclareLaunchArgument(
                "gz_ip",
                default_value=DEFAULT_GZ_IP,
                description="Gazebo IP (GZ_IP)",
            ),
            DeclareLaunchArgument(
                "world_ready_timeout",
                default_value="30.0",
                description="Seconds to wait for the Gazebo scene/info service",
            ),
            DeclareLaunchArgument(
                "micro_xrce_port",
                default_value="8888",
                description="UDP port shared by PX4 and the Micro XRCE-DDS Agent",
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
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            SetEnvironmentVariable(
                "ROS_LOCALHOST_ONLY",
                LaunchConfiguration("ros_localhost_only"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
