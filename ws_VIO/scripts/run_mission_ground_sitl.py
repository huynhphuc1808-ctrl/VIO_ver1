#!/usr/bin/env python3
"""Bounded native prearm and live-file checks in disarmed PX4 SITL.

The only flight commands are individual nonforced negative arm probes, gated
by the existing probe's fresh disarmed/landed/native-rejection prerequisites.
There is no mode-selection, takeoff or successful-arm path.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess as sp
import sys
import time

from collect_phase1_sitl import atomic_json, utc_now


def relay():
    import rclpy
    from px4_msgs.msg import VehicleLocalPosition
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node("astra_ground_local_relay")
    publisher = node.create_publisher(VehicleLocalPosition, "/astra/ground_local_position", 10)
    marker = (Path(sys.argv[sys.argv.index("--epoch-reset-file") + 1])
              if "--epoch-reset-file" in sys.argv else None)
    announced = False
    clock_delay = (float(sys.argv[sys.argv.index("--delay-clock-seconds") + 1])
                   if "--delay-clock-seconds" in sys.argv else None)
    clock_started = time.monotonic()
    clock_publisher = None
    clock_subscription = None
    if clock_delay is not None:
        clock_publisher = node.create_publisher(Clock, "/astra/ground_clock", 10)
        clock_subscription = node.create_subscription(Clock, "/clock", lambda message:
            clock_publisher.publish(message) if time.monotonic() - clock_started >= clock_delay else None,
            qos_profile_sensor_data)

    def forward(message):
        nonlocal announced
        if marker is not None and marker.exists():
            old = message.xy_reset_counter
            message.xy_reset_counter = (old + 1) % 256
            if not announced:
                print(json.dumps({"test_fault": "mission_only_epoch_reset", "native_timestamp": message.timestamp,
                                  "original_xy_reset_counter": old, "injected_xy_reset_counter": message.xy_reset_counter,
                                  "note": "Only the mission alias is modified; PX4 estimator/EV input remains untouched."}), flush=True)
                announced = True
        publisher.publish(message)

    subscription = node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                             forward, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    workspace = Path(__file__).resolve().parents[1]
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "42"), ROS_LOCALHOST_ONLY="1",
               GZ_PARTITION="uav_vio", GZ_IP="127.0.0.1", PX4_PARAM_SDLOG_DIRS_MAX="0",
               OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="1")
    children, checks = [], {}
    mission_file = output / "mission.json"
    fixture = json.loads((workspace / "src/uav_vio_mission/config/local_takeoff_land.json").read_text())
    atomic_json(mission_file, fixture)

    def event(kind, **fields):
        row = dict(event=kind, utc=utc_now(), wall=time.monotonic(), **fields)
        with (output / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(row) + "\n")
        print(json.dumps(row), flush=True)

    def launch(name, command):
        stream = (output / (name + ".log")).open("w")
        process = sp.Popen(command, cwd=workspace, stdout=stream, stderr=sp.STDOUT,
                           env=dict(env, ROS_LOG_DIR=str(output / ("ros_" + name))), start_new_session=True)
        children.append((name, process, stream))
        event("started", name=name, pid=process.pid)
        return process

    def latest():
        try:
            return json.loads((output / "collector/latest.json").read_text())
        except (FileNotFoundError, ValueError):
            return {"topics": {}}

    def state(snapshot):
        return snapshot["topics"].get("/uav_vio/mission/status", {}).get("parsed_json", {})

    def wait(name, predicate, timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            snapshot = latest()
            if predicate(snapshot):
                atomic_json(output / (name + ".json"), snapshot)
                event(name)
                return snapshot
            if any("vehicle_status" in topic and row["message"].get("arming_state") == 2
                   for topic, row in snapshot["topics"].items()):
                raise RuntimeError("Unexpected arm in ground episode")
            time.sleep(.1)
        raise RuntimeError("Timed out: " + name)

    def command(name, command):
        with (output / (name + ".log")).open("w") as stream:
            result = sp.run(command, cwd=workspace, env=env, stdout=stream, stderr=sp.STDOUT, timeout=30)
        checks[name] = result.returncode == 0
        event(name, exit_code=result.returncode)
        return result.returncode

    def probe(label, gate):
        if command(label, [sys.executable, str(workspace / "scripts/probe_phase1_prearm.py"),
                "--output", str(output / (label + ".json")), "--label", label, "--gate-id", str(gate)]) != 0:
            raise RuntimeError("Native negative prearm probe failed: " + label)

    error = None
    try:
        launch("collector", [sys.executable, str(workspace / "scripts/collect_phase1_sitl.py"),
                             "--output", str(output / "collector"), "--duration", "360",
                             "--snapshot-interval", "0.1"])
        launch("base", ["ros2", "launch", "uav_vio_bringup", "x500_d435i.launch.py", "world:=kthspacelab",
                         "gui:=false", "rviz:=false", "vio_estimator:=false", "px4_parameter_profile:=vio_only"])
        wait("native_telemetry", lambda snap: any("vehicle_status" in t for t in snap["topics"]), 75)
        for label, text in (("empty", '{"version":1,"mission":{"items":[]}}'), ("malformed", "{")):
            invalid = output / (label + ".json")
            invalid.write_text(text)
            code = command("startup_" + label, ["ros2", "run", "uav_vio_mission", "vio_mission",
                            "--ros-args", "-p", "mission_file:=" + str(invalid)])
            checks["startup_" + label] = code == 1
            if code != 1:
                raise RuntimeError("Invalid startup unexpectedly accepted")
        local_relay = launch("local_relay", [sys.executable, str(Path(__file__).resolve()), "--relay",
                                             "--delay-clock-seconds", "12"])
        launch("mission", ["ros2", "run", "uav_vio_mission", "vio_mission", "--ros-args", "-p", "use_sim_time:=true",
                           "-p", "auto_start:=false", "-p", "mission_file:=" + str(mission_file),
                           "-r", "/fmu/out/vehicle_local_position_v1:=/astra/ground_local_position",
                           "-r", "/clock:=/astra/ground_clock"])
        snap = wait("registered_without_vio", lambda s: state(s).get("registered"))
        time.sleep(1)
        snap = latest()
        checks["zero_clock_warmup_no_crash_no_arm"] = state(snap).get("clock_ready") is False and not state(snap).get("armed")
        checks["initial_monitor_no_spurious_update_error"] = not state(snap).get("update_error")
        local = next(r["message"] for t, r in snap["topics"].items() if "vehicle_local_position" in t)
        checks["local_only_registration"] = not local["xy_global"] and not local["z_global"] and local["ref_timestamp"] == 0
        gate = next(r["message"] for t, r in snap["topics"].items() if "register_ext_component_reply" in t)
        if bytes(gate["name"]).split(b"\0")[0] != b"VIO Mission Gate":
            raise RuntimeError("Latest registration is not the independent VIO Mission Gate")
        probe("prearm_no_vio", gate["arming_check_id"])
        wait("clock_warmup_ready", lambda s: state(s).get("clock_ready"))
        vio = launch("vio_estimator", ["ros2", "launch", "uav_vio_bringup", "vio_estimator.launch.py"])
        wait("ready_with_genuine_vio", lambda s: state(s).get("ready"), 90)
        original = state(latest())["revision"]
        mission_file.write_text("{")
        wait("malformed_live_status", lambda s: "parse_error" in state(s).get("update_error", ""))
        checks["malformed_preserves_revision"] = state(latest())["revision"] == original
        atomic_json(mission_file, fixture)
        wait("same_revision_recovery", lambda s: state(s).get("revision") == original and
             not state(s).get("update_error") and not state(s).get("started"))
        checks["same_revision_restoration_clears_error_without_start"] = True
        fixture["mission"]["items"][2]["duration_s"] = 3.0
        atomic_json(mission_file, fixture)
        wait("inactive_valid_reload", lambda s: state(s).get("revision") != original and not state(s).get("update_error"))
        revision = state(latest())["revision"]
        unknown = json.loads(json.dumps(fixture))
        unknown["mission"]["items"][1]["type"] = "unknownAction"
        atomic_json(mission_file, unknown)
        wait("unknown_action_live_status", lambda s: "unsupported action" in state(s).get("update_error", ""))
        checks["unknown_preserves_revision"] = state(latest())["revision"] == revision
        fixture["mission"]["items"][2]["duration_s"] = 4.0
        atomic_json(mission_file, fixture)
        wait("second_inactive_reload", lambda s: state(s).get("revision") != revision and not state(s).get("update_error"))
        wait("ready_before_local_fault", lambda s: state(s).get("ready"))
        os.killpg(local_relay.pid, signal.SIGSTOP)
        wait("local_stale", lambda s: state(s).get("ready") is False)
        probe("prearm_stale_local", gate["arming_check_id"])
        os.killpg(local_relay.pid, signal.SIGCONT)
        wait("local_recovered", lambda s: state(s).get("ready"))
        os.killpg(vio.pid, signal.SIGSTOP)
        wait("vio_stale", lambda s: state(s).get("ready") is False)
        probe("prearm_stale_vio", gate["arming_check_id"])
        checks["no_automatic_start"] = not state(latest()).get("started") and not state(latest()).get("armed")
    except BaseException as exception:
        error = repr(exception)
        event("ground_failed", error=error)
        atomic_json(output / "failure_snapshot.json", latest())
    finally:
        shutdown = []
        for name, process, stream in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGCONT)
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except sp.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            shutdown.append(dict(name=name, pid=process.pid, exit_code=process.returncode))
            stream.close()
        atomic_json(output / "result.json", {"pass": error is None and bool(checks) and all(checks.values()),
                    "error": error, "checks": checks, "shutdown": shutdown, "ended_utc": utc_now()})
    return 0 if error is None and all(checks.values()) else 1


if __name__ == "__main__":
    if "--relay" in sys.argv:
        relay()
    else:
        raise SystemExit(main())
