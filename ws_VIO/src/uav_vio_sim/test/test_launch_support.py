"""Regression checks for the shared Gazebo/PX4 world contract."""

import codecs
import shutil

import pytest

from uav_vio_sim.launch_support import (
    _gz_shell_setup,
    _prepare_runtime_world,
    _read_world_name,
    _resolve_world_path,
)


@pytest.mark.parametrize("bom_count", [0, 1, 2])
def test_world_name_and_runtime_copy_preserve_source(tmp_path, bom_count):
    """Use the direct world name for readiness even when basename differs."""
    path = tmp_path / "different filename.sdf"
    xml = b'<?xml version="1.0"?><sdf><world name="actual_world"/></sdf>'
    source = codecs.BOM_UTF8 * bom_count + xml
    path.write_bytes(source)
    runtime, directory, count, name = _prepare_runtime_world(path)
    try:
        assert name == _read_world_name(path) == "actual_world"
        assert count == bom_count
        assert runtime.read_bytes() == xml
        assert path.read_bytes() == source
        assert (directory is None) == (bom_count == 0)
        assert (runtime == path) == (bom_count == 0)
    finally:
        if directory:
            shutil.rmtree(directory)


@pytest.mark.parametrize("xml", [
    b"<sdf>",
    b"<sdf><model><world name='nested'/></model></sdf>",
    b"<sdf><world name='a'/><world name='b'/></sdf>",
    b"<sdf><world name=' '/></sdf>",
])
def test_invalid_world_rejected_by_both_consumers(tmp_path, xml):
    """Server and spawn must reject the same malformed or ambiguous inputs."""
    path = tmp_path / "world.sdf"
    path.write_bytes(xml)
    for reader in (_prepare_runtime_world, _read_world_name):
        with pytest.raises(RuntimeError):
            reader(path)


def test_named_and_absolute_world_resolution(tmp_path):
    """Keep installed named worlds and absolute external SDF support."""
    path = tmp_path / "world.sdf"
    path.touch()
    for value in ("world", "world.sdf", str(path)):
        assert _resolve_world_path(pkg_worlds_dir=tmp_path, world=value) == path
    with pytest.raises(RuntimeError, match="not found"):
        _resolve_world_path(pkg_worlds_dir=tmp_path, world="missing")


def test_gazebo_environment_shell_quoting(tmp_path):
    """Treat spaces and shell metacharacters in user paths as literal values."""
    import subprocess

    env_file = tmp_path / "gz env.sh"
    env_file.write_text("export GZ_SIM_RESOURCE_PATH=upstream\n")
    setup = _gz_shell_setup(
        px4_dir=str(tmp_path), gz_env_sh=str(env_file),
        pkg_models_dir="models $(false)", pkg_worlds_dir="world's directory",
        px4_plugins_dir="plugins; false", px4_server_config="server config",
        gz_partition="partition with spaces", gz_ip="127.0.0.1",
    )
    result = subprocess.run(
        ["bash", "-c", setup + " && env"], check=True, capture_output=True, text=True
    )
    assert "GZ_SIM_RESOURCE_PATH=models $(false):world's directory:upstream" in result.stdout
    assert "GZ_PARTITION=partition with spaces" in result.stdout
    assert "GZ_SIM_SERVER_CONFIG_PATH=server config" in result.stdout


def test_vio_only_profile_disables_gnss_and_keeps_local_vision_fusion():
    from pathlib import Path

    profile = Path(__file__).parents[1] / "config" / "vio_only.params"
    values = {
        fields[0]: fields[1]
        for line in profile.read_text().splitlines()
        if (fields := line.split()) and not fields[0].startswith("#")
    }
    assert values["EKF2_GPS_CTRL"] == "0"
    assert values["SYS_HAS_GPS"] == "0"
    assert values["SIM_GZ_EN_GPS"] == "0"
    assert values["SIM_GPS_USED"] == "0"
    assert values["SENS_EN_GPSSIM"] == "0"
    assert values["EKF2_EV_CTRL"] == "7"
    assert values["EKF2_HGT_REF"] == "3"
    assert values["NAV_DLL_ACT"] == "0"
    assert "COM_POS_FS_EPH" in values and "EKF2_NOAID_TOUT" in values


def test_circle_launch_selects_vio_profile_and_single_mission_executor():
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    path = Path(__file__).parents[2] / "uav_vio_bringup" / "launch" / "vio_circle.launch.py"
    spec = importlib.util.spec_from_file_location("circle_launch_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    public = module.generate_launch_description()
    assert len(public.entities) == 1
    assert isinstance(public.entities[0], IncludeLaunchDescription)
    # Inspect the composed launch without executing any process or ROS node.
    generic = public.entities[0].launch_description_source.get_launch_description(context)
    nodes = [entity for entity in generic.entities if isinstance(entity, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_package == "uav_vio_mission"
    assert nodes[0].node_executable == "vio_mission"
    base = [entity for entity in generic.entities if isinstance(entity, IncludeLaunchDescription)]
    assert len(base) == 1
    assert dict(base[0].launch_arguments)["px4_parameter_profile"] == "vio_only"
    arguments = {entity.name: entity for entity in generic.entities if isinstance(entity, DeclareLaunchArgument)}
    assert perform_substitutions(context, arguments["auto_start"].default_value) == "false"
    assert perform_substitutions(context, arguments["mission_file"].default_value) == ""
