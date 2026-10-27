"""Shared world preparation and Gazebo/PX4 environment contract."""

import codecs
from pathlib import Path
import shlex
import tempfile
import xml.etree.ElementTree as ET


def _resolve_world_path(*, pkg_worlds_dir: Path, world: str) -> Path:
    world_path = Path(world).expanduser()
    if not world_path.is_absolute():
        world_name = world if world.endswith(".sdf") else f"{world}.sdf"
        world_path = pkg_worlds_dir / world_name
    if not world_path.is_file():
        raise RuntimeError(f"Gazebo world not found: {world_path}")
    return world_path


def _parse_world_name(world_data: bytes, world_path: Path) -> str:
    try:
        sdf_root = ET.fromstring(world_data)
    except ET.ParseError as exc:
        raise RuntimeError(f"Invalid Gazebo world XML {world_path}: {exc}") from exc

    world_elements = [
        element
        for element in sdf_root
        if element.tag.rsplit("}", maxsplit=1)[-1] == "world"
    ]
    if len(world_elements) != 1:
        raise RuntimeError(
            f"Gazebo world {world_path} must contain exactly one <world> element"
        )
    world_name = world_elements[0].get("name", "").strip()
    if not world_name:
        raise RuntimeError(f"Gazebo world has no non-empty name: {world_path}")
    return world_name


def _prepare_runtime_world(world_path: Path) -> tuple[Path, Path | None, int, str]:
    """Strip leading UTF-8 BOM markers in a temporary copy, if present."""
    world_data, bom_count, world_name = _read_world(world_path)

    if bom_count == 0:
        return world_path, None, bom_count, world_name

    runtime_dir = Path(tempfile.mkdtemp(prefix="uav_vio_world_"))
    runtime_world = runtime_dir / world_path.name
    runtime_world.write_bytes(world_data)
    return runtime_world, runtime_dir, bom_count, world_name


def _gz_shell_setup(
    *,
    px4_dir: str,
    gz_env_sh: str,
    pkg_models_dir: str,
    pkg_worlds_dir: str,
    px4_plugins_dir: str,
    px4_server_config: str,
    gz_partition: str,
    gz_ip: str,
) -> str:
    return (
        f"cd {shlex.quote(px4_dir)}"
        f" && source {shlex.quote(gz_env_sh)}"
        f" && export GZ_PARTITION={shlex.quote(gz_partition)}"
        f" && export GZ_IP={shlex.quote(gz_ip)}"
        f" && export GZ_SIM_RESOURCE_PATH={shlex.quote(pkg_models_dir)}:"
        f"{shlex.quote(pkg_worlds_dir)}:$GZ_SIM_RESOURCE_PATH"
        f" && export GZ_SIM_SYSTEM_PLUGIN_PATH={shlex.quote(px4_plugins_dir)}:"
        "$GZ_SIM_SYSTEM_PLUGIN_PATH"
        f" && export GZ_SIM_SERVER_CONFIG_PATH={shlex.quote(px4_server_config)}"
        " && export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH"
    )


def _read_world(world_path: Path) -> tuple[bytes, int, str]:
    """Read and validate a source world without changing it."""
    world_data = world_path.read_bytes()
    bom_count = 0
    while world_data.startswith(codecs.BOM_UTF8):
        world_data = world_data[len(codecs.BOM_UTF8):]
        bom_count += 1
    return world_data, bom_count, _parse_world_name(world_data, world_path)


def _read_world_name(world_path: Path) -> str:
    """Return the direct SDF world name, including for multi-BOM files."""
    return _read_world(world_path)[2]
