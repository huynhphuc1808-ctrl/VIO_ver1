import shlex
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from uav_vio_sim.launch_support import (
    _gz_shell_setup,
    _prepare_runtime_world,
    _resolve_world_path,
)


DEFAULT_GZ_WORLD = "lawn"
DEFAULT_GZ_VERBOSE = "1"
DEFAULT_GZ_PARTITION = "uav_vio"
DEFAULT_GZ_IP = "127.0.0.1"


def _cleanup_runtime_world(context, *, runtime_dir: str):
    del context
    shutil.rmtree(runtime_dir, ignore_errors=True)
    return []


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser()
    world = LaunchConfiguration("world").perform(context)
    gz_verbose = LaunchConfiguration("gz_verbose").perform(context)
    gz_partition = LaunchConfiguration("gz_partition").perform(context)
    gz_ip = LaunchConfiguration("gz_ip").perform(context)
    headless = not IfCondition(LaunchConfiguration("gui")).evaluate(context)

    if not px4_dir.is_dir():
        raise RuntimeError(
            "px4_dir is empty or invalid. Set env PX4_DIR or pass "
            "px4_dir:=/path/to/PX4-Autopilot"
        )

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
    for required in (gz_env_sh, px4_plugins_dir, px4_server_config):
        if not required.exists():
            raise RuntimeError(f"Required PX4 Gazebo artifact not found: {required}")

    pkg_share_dir = Path(get_package_share_directory("uav_vio_sim"))
    pkg_models_dir = pkg_share_dir / "models"
    pkg_worlds_dir = pkg_share_dir / "worlds"
    source_world = _resolve_world_path(pkg_worlds_dir=pkg_worlds_dir, world=world)
    runtime_world, runtime_dir, bom_count, world_name = _prepare_runtime_world(
        source_world
    )

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
    gz_server_cmd = (
        f"{gz_shell_setup}"
        f" && gz sim --verbose={shlex.quote(gz_verbose)} -r -s "
        f"{'--headless-rendering ' if headless else ''}"
        f"{shlex.quote(str(runtime_world))}"
    )
    gz_gui_cmd = f"{gz_shell_setup} && gz sim -g"

    gz_server = ExecuteProcess(
        cmd=["bash", "-c", gz_server_cmd],
        output="screen",
    )
    start_gui_after_server = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_server,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=["bash", "-c", gz_gui_cmd],
                            output="screen",
                            condition=IfCondition(LaunchConfiguration("gui")),
                        )
                    ],
                )
            ],
        )
    )
    shutdown_after_server = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_server,
            on_exit=[
                LogInfo(msg="Gazebo server exited; shutting down simulation."),
                EmitEvent(event=Shutdown(reason="Gazebo server exited")),
            ],
        )
    )

    actions = [start_gui_after_server, shutdown_after_server]
    if runtime_dir is not None:
        actions.insert(
            0,
            LogInfo(
                msg=(
                    f"Removed {bom_count} leading UTF-8 BOM marker(s) from "
                    f"{source_world} in temporary runtime copy {runtime_world}; "
                    "the source world was not modified."
                )
            ),
        )
        actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(
                            function=_cleanup_runtime_world,
                            kwargs={"runtime_dir": str(runtime_dir)},
                        )
                    ]
                )
            )
        )
    actions.extend(
        [
            LogInfo(msg=f"Starting Gazebo world '{world_name}' from {runtime_world}"),
            gz_server,
        ]
    )
    return actions


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
                description="Gazebo world name, SDF basename, or absolute SDF path",
            ),
            DeclareLaunchArgument(
                "gz_verbose",
                default_value=DEFAULT_GZ_VERBOSE,
                description="Gazebo verbosity level for `gz sim --verbose`",
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
                "gui",
                default_value="true",
                description="If true: start the Gazebo GUI after the server process",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
