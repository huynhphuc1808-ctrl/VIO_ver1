#!/usr/bin/env python3
"""Run one bounded, explicitly requested JSON-mission acceptance in PX4 SITL.

Requires --fly, PX4_DIR and the sourced project overlay. Each episode starts a
fresh genuine sensor/VIO stack, checks stationary VIO and native fusion before
starting its sole mission backend, and preserves collector, graph and ULog.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

from collect_phase1_sitl import atomic_json, plain, utc_now, versioned_topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fly", required=True, action="store_true")
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=360)
    parser.add_argument("--fault", choices=("none", "vio-loss", "stale-local", "takeover", "reselection", "process-death", "active-edit", "progress-timeout", "frame-reset"), default="none")
    parser.add_argument("--fault-action", default="localHold")
    parser.add_argument("--screenshot", action="store_true", help="Capture the real Gazebo GUI during an airborne Hold >= 8 s")
    parser.add_argument("--entrypoint", choices=("executable", "circle"), default="executable",
                        help="Use executable or the public circle launch with explicit manual selection")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 30 <= args.timeout <= 1800:
        parser.error("--timeout must be finite, between 30 and 1800 wall seconds")
    if args.entrypoint == "circle" and (args.fault != "none" or args.screenshot):
        parser.error("public circle entrypoint check is a nominal manual-activation episode")
    if args.screenshot:
        items = json.loads(args.mission.read_text())["mission"]["items"]
        if not any(i["type"] == "localHold" and i.get("duration_s", 0) >= 8 for i in items):
            parser.error("--screenshot requires an airborne localHold of at least 8 seconds")
    workspace = Path(__file__).resolve().parents[1]
    px4 = Path(os.environ["PX4_DIR"]).resolve()
    if not (px4 / "build/px4_sitl_default/bin/px4").is_file():
        parser.error("PX4_DIR must identify the built, pinned PX4 SITL checkout")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.mission, output / "mission.json")
    shutil.copy2(args.mission, output / "definition_before_run.json")
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "42"),
               ROS_LOCALHOST_ONLY="1", GZ_PARTITION="uav_vio", GZ_IP="127.0.0.1",
               PX4_PARAM_SDLOG_DIRS_MAX="0", OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="1")
    # PX4 SITL persists generated parameters between starts. Reset this test's
    # timeout override on normal episodes (unless the caller explicitly sets it).
    env.setdefault("PX4_PARAM_MPC_Z_VEL_MAX_UP", "3.0")
    env.setdefault("PX4_PARAM_MPC_Z_V_AUTO_UP", "3.0")
    if args.fault == "progress-timeout":
        # Legal pinned PX4 climb limit for this deliberate deadline test only.
        # The supplied fixture must request an unreachable-within-deadline Hold
        # altitude. Native Land and all health/freshness gates stay unchanged.
        env["PX4_PARAM_MPC_Z_VEL_MAX_UP"] = "0.5"
        env["PX4_PARAM_MPC_Z_V_AUTO_UP"] = "0.5"
    if args.screenshot:
        env["QT_QPA_PLATFORM"] = "xcb"
        env["GZ_SIM_RESOURCE_PATH"] = ":".join([
            str(workspace / "src/uav_vio_sim/models"), str(workspace / "src/uav_vio_sim/worlds"),
            str(px4 / "Tools/simulation/gz/models"), str(px4 / "Tools/simulation/gz/worlds"),
            env.get("GZ_SIM_RESOURCE_PATH", "")])
    processes = []
    paused = []
    result = {"schema": "json-mission-sitl-episode-v1", "success": False,
              "mission_sha256": hashlib.sha256((output / "mission.json").read_bytes()).hexdigest(),
              "started_utc": utc_now(), "scope": "SITL only", "fault": args.fault,
              "fault_action": args.fault_action}
    result["entrypoint"] = args.entrypoint
    result["bootstrap_source_component"] = 191 if args.entrypoint == "circle" else 1
    result["test_climb_limit_m_s"] = float(env["PX4_PARAM_MPC_Z_VEL_MAX_UP"])
    result["source_sha256"] = {name: hashlib.sha256((workspace / name).read_bytes()).hexdigest()
        for name in ("src/uav_vio_mission/src/mission_actions.cpp", "src/uav_vio_mission/src/mission_runtime.cpp",
                     "src/uav_vio_mission/src/mission_context.cpp", "src/uav_vio_mission/src/mission_definition.cpp",
                     "src/uav_vio_sim/models/d435i_sensor/model.sdf", "src/uav_vio_sim/config/vio_only.params",
                     "src/uav_vio_bringup/launch/vio_circle.launch.py", "src/uav_vio_bringup/launch/vio_mission.launch.py",
                     "scripts/run_mission_sitl.py", "scripts/collect_phase1_sitl.py")}
    from ament_index_python.packages import get_package_prefix
    executable = Path(get_package_prefix("uav_vio_mission")) / "lib/uav_vio_mission/vio_mission"
    result["mission_binary"] = {"path": str(executable.resolve()),
                                "sha256": hashlib.sha256(executable.read_bytes()).hexdigest()}

    def event(kind, **fields):
        record = dict(event=kind, utc=utc_now(), wall=time.monotonic(), **fields)
        with (output / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(record) + "\n")
        print(json.dumps(record), flush=True)

    def launch(name, command):
        stream = (output / (name + ".log")).open("w")
        process = subprocess.Popen(command, cwd=workspace,
            env=dict(env, ROS_LOG_DIR=str(output / ("ros_" + name))),
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process, stream))
        event("process_started", name=name, pid=process.pid, command=command)
        return process

    def latest():
        try:
            return json.loads((output / "collector/latest.json").read_text())
        except (FileNotFoundError, ValueError):
            return {"topics": {}}

    def message(snapshot, name):
        return next((row["message"] for topic, row in snapshot["topics"].items()
                     if topic.rsplit("/", 1)[-1] == name or
                     re.fullmatch(re.escape(name) + r"_v\d+", topic.rsplit("/", 1)[-1])), {})

    def status(snapshot):
        return snapshot["topics"].get("/uav_vio/mission/status", {}).get("parsed_json", {})

    def wait_for(name, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            snapshot = latest()
            if predicate(snapshot):
                atomic_json(output / (name + ".json"), snapshot)
                event(name)
                return snapshot
            for child_name, child, _ in processes:
                if child.poll() is not None:
                    raise RuntimeError(f"{child_name} exited {child.returncode} during {name}")
            time.sleep(0.1)
        raise RuntimeError("Timed out: " + name)

    def stationary(snapshot):
        now = time.monotonic()
        for token, ttl in (("/vio/odometry", .5), ("/vio/status", .5),
                           ("/vehicle_local_position", .5), ("/vehicle_status", 2),
                           ("/estimator_status_flags", 5)):
            rows = [row for topic, row in snapshot["topics"].items() if token in topic]
            if not rows or now - rows[0]["receipt_steady_s"] >= ttl:
                return False
        local = message(snapshot, "vehicle_local_position")
        fusion = message(snapshot, "estimator_status_flags")
        vio = message(snapshot, "odometry")
        velocity = vio.get("twist", {}).get("twist", {}).get("linear", {})
        return (message(snapshot, "vehicle_status").get("arming_state") == 1 and
                message(snapshot, "vehicle_land_detected").get("landed") is True and
                all(local.get(key) is True for key in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")) and
                not local.get("xy_global") and not local.get("z_global") and local.get("ref_timestamp") == 0 and
                all(fusion.get(key) is True for key in ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt")) and
                not fusion.get("cs_gnss_pos") and not fusion.get("cs_gnss_vel") and
                snapshot["topics"].get("/vio/status", {}).get("message", {}).get("data") == "VALID_ODOMETRY" and
                math.sqrt(sum(velocity.get(k, 100) ** 2 for k in ("x", "y", "z"))) < .15 and
                math.sqrt(sum(local.get(k, 100) ** 2 for k in ("vx", "vy", "vz"))) < .2)

    def gui_service(name, kind, request):
        response = subprocess.run(["gz", "service", "-s", name, "--reqtype", kind,
            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request],
            env=env, capture_output=True, text=True, timeout=5)
        if response.returncode or "true" not in response.stdout:
            raise RuntimeError("Gazebo GUI service failed: " + name + " " + response.stderr)
        return response.stdout

    def operator_select(mode_id, reselect=True):
        # An ephemeral operator intent for this explicit SITL fault test, never
        # an arm request or a second setpoint/controller backend.
        import rclpy
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck
        from rclpy.qos import qos_profile_sensor_data
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(domain_id=int(env["ROS_DOMAIN_ID"]))
        node = rclpy.create_node("astra_operator_reselection")
        publisher = node.create_publisher(VehicleCommand, versioned_topic("/fmu/in", "vehicle_command", VehicleCommand), 10)
        replies = []
        subscription = node.create_subscription(VehicleCommandAck,
            versioned_topic("/fmu/out", "vehicle_command_ack", VehicleCommandAck),
            lambda msg: replies.append(plain(msg)) if msg.command == 100001 and msg.target_component == 191 else None,
            qos_profile_sensor_data)
        try:
            end = time.monotonic() + 4
            while publisher.get_subscription_count() < 2 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            snapshot = latest()
            state = status(snapshot)
            eligible = ((message(snapshot, "vehicle_status").get("arming_state") == 2 and
                         state.get("phase") in ("failed", "interrupted")) if reselect else
                        (message(snapshot, "vehicle_status").get("arming_state") == 1 and
                         state.get("ready") and state.get("landed") and not state.get("started")))
            if not 23 <= mode_id <= 30 or publisher.get_subscription_count() < 2 or not eligible:
                raise RuntimeError("Reselection prerequisites missing; no operator command sent")
            command = VehicleCommand()
            command.command = 100001
            command.param1 = float(mode_id)
            command.target_system = command.source_system = 1
            command.target_component = 1
            command.source_component = 191
            command.from_external = True
            publisher.publish(command)
            end = time.monotonic() + 4
            while not replies and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            atomic_json(output / ("operator_reselection.json" if reselect else "operator_selection.json"), {"before": snapshot,
                        "command": plain(command), "acknowledgements": replies})
            if not replies or replies[-1]["result"] != 0:
                raise RuntimeError("Native PX4 did not accept the controlled operator reselection")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    try:
        launch("collector", [sys.executable, str(workspace / "scripts/collect_phase1_sitl.py"),
                             "--output", str(output / "collector"), "--duration", str(args.timeout + 240),
                             "--snapshot-interval", "0.1"])
        base = launch("base", ["ros2", "launch", "uav_vio_bringup",
            "vio_circle.launch.py" if args.entrypoint == "circle" else "x500_d435i.launch.py",
            "world:=kthspacelab", "gui:=false", "rviz:=false", *(
                ["auto_start:=false"] if args.entrypoint == "circle" else ["px4_parameter_profile:=vio_only"])])
        if args.screenshot:
            gui_config = Path.home() / ".gz/sim/8/gui.config"
            text = gui_config.read_text()
            text = re.sub(r"(<start_paused>)\s*true\s*(</start_paused>)", r"\1false\2", text)
            (output / "gui.config").write_text(text)
            launch("gui", ["gz", "sim", "-g", "--gui-config", str(output / "gui.config")])
        stable = {"since": None, "origin": None}

        def stable_window(snapshot):
            if not stationary(snapshot):
                stable["since"] = None
                return False
            local = message(snapshot, "vehicle_local_position")
            position = [local[k] for k in ("x", "y", "z")]
            if stable["since"] is None or math.dist(position, stable["origin"]) > .15:
                stable.update(since=time.monotonic(), origin=position)
            return time.monotonic() - stable["since"] >= 3

        wait_for("stationary_prearm", stable_window, 120)
        graph = json.loads((output / "collector/graph_latest.json").read_text())
        command_endpoints = {topic: entry.get("publishers", []) for topic, entry in graph["monitored"].items()
                             if "/in/vehicle_command" in topic or "/in/goto_setpoint" in topic}
        if args.entrypoint == "executable" and any(command_endpoints.values()):
            raise RuntimeError("Flight-command backend already present before mission start")
        if args.entrypoint == "circle" and (not any(command_endpoints.values()) or any(
            entry["node_name"] != "uav_vio_mission" for entries in command_endpoints.values() for entry in entries)):
            raise RuntimeError("Public circle launch did not select exactly the generic mission backend")
        atomic_json(output / "prearm_graph.json", graph)
        if args.screenshot:
            x, y, z = 1.8, -2.5, 1.6
            yaw = math.atan2(-y, -x)
            pitch = math.atan2(z - 1.8, math.hypot(x, y))
            q = (-math.sin(pitch / 2) * math.sin(yaw / 2), math.sin(pitch / 2) * math.cos(yaw / 2),
                 math.cos(pitch / 2) * math.sin(yaw / 2), math.cos(pitch / 2) * math.cos(yaw / 2))
            gui_service("/gui/move_to/pose", "gz.msgs.GUICamera",
                f"pose: {{position: {{x:{x},y:{y},z:{z}}},orientation: {{x:{q[0]},y:{q[1]},z:{q[2]},w:{q[3]}}}}}")
        extra = []
        relay_process = None
        if args.fault in ("stale-local", "frame-reset"):
            relay_process = launch("local_relay", [sys.executable,
                str(workspace / "scripts/run_mission_ground_sitl.py"), "--relay",
                "--epoch-reset-file", str(output / "inject_epoch_reset")])
            extra = ["-r", "/fmu/out/vehicle_local_position_v1:=/astra/ground_local_position"]
        mission_process = base if args.entrypoint == "circle" else launch("mission", ["ros2", "run", "uav_vio_mission", "vio_mission", "--ros-args",
                           "-p", "use_sim_time:=true", "-p", "auto_start:=true",
                           "-p", "mission_file:=" + str(output / "mission.json"), *extra])
        wait_for("registered", lambda snap: status(snap).get("registered"), 60)
        if args.entrypoint == "circle":
            ready = wait_for("manual_ready", lambda snap: status(snap).get("ready"), 30)
            validation = subprocess.run([str(executable), "--validate-mission", str(output / "mission.json")],
                                        capture_output=True, text=True, timeout=5)
            if validation.returncode or "revision=" + status(ready)["revision"] not in validation.stdout:
                raise RuntimeError("Public generated circle definition differs from the reference fixture")
            operator_select(status(ready)["mode_id"], reselect=False)
        wait_for("armed", lambda snap: status(snap).get("armed") and status(snap).get("started"), 60)
        previous, deadline = None, time.monotonic() + args.timeout
        injected = False
        captured = False
        revision = status(latest()).get("revision")
        while time.monotonic() < deadline:
            snapshot = latest()
            state = status(snapshot)
            progress = (state.get("index"), state.get("action"), state.get("phase"))
            if progress != previous:
                event("progress", status=state)
                previous = progress
            if args.screenshot and not captured and state.get("action") == "localHold" and state.get("armed") and not state.get("landed"):
                folder = output / "gui_captures"
                folder.mkdir()
                before = latest()
                response = gui_service("/gui/screenshot", "gz.msgs.StringMsg", "data: " + json.dumps(str(folder)))
                images = list(folder.glob("*.png"))
                image_deadline = time.monotonic() + 3
                while not images and time.monotonic() < image_deadline:
                    time.sleep(.05)
                    images = list(folder.glob("*.png"))
                if not images:
                    raise RuntimeError("Screenshot service succeeded without a PNG artifact")
                image = max(images, key=lambda p: p.stat().st_mtime)
                shutil.copy2(image, output / "airborne.png")
                atomic_json(output / "airborne_snapshot.json", {"before": before, "after": latest(),
                            "service_response": response, "capture_utc": utc_now(),
                            "image_sha256": hashlib.sha256((output / "airborne.png").read_bytes()).hexdigest()})
                captured = True
                event("airborne_gui_screenshot", path=str(output / "airborne.png"))
            if not injected and args.fault != "none" and state.get("action") == args.fault_action and not state.get("landed"):
                if not state.get("armed") or not state.get("command_owner"):
                    raise RuntimeError("Fault injection requires fresh armed owned episode")
                atomic_json(output / "before_fault.json", snapshot)
                result["fault_wall_s"] = time.monotonic()
                result["fault_native_timestamp_us"] = message(snapshot, "vehicle_local_position").get("timestamp")
                if args.fault == "vio-loss":
                    text = (output / "base.log").read_text(errors="replace")
                    matches = re.findall(r"\[run_subscribe_msckf-\d+\]: process started with pid \[(\d+)\]", text)
                    if len(matches) != 1:
                        raise RuntimeError("Cannot uniquely identify owned VIO_ver1 process")
                    pid = int(matches[0])
                    # Verify ancestry rather than stopping an unrelated estimator.
                    parent = pid
                    while parent > 1 and parent != base.pid:
                        fields = Path(f"/proc/{parent}/status").read_text().splitlines()
                        parent = int(next(line.split()[1] for line in fields if line.startswith("PPid:")))
                    if parent != base.pid:
                        raise RuntimeError("VIO_ver1 PID is not a child of this bringup")
                    os.kill(pid, signal.SIGSTOP)
                    paused.append(pid)
                elif args.fault == "stale-local":
                    os.killpg(relay_process.pid, signal.SIGSTOP)
                    paused.append(relay_process.pid)
                elif args.fault == "frame-reset":
                    if not state.get("frame_valid") or state.get("frame_initializing"):
                        raise RuntimeError("Epoch reset test requires the frozen mission frame")
                    (output / "inject_epoch_reset").write_text("Mission-only telemetry epoch fault; not PX4/EV input.\n")
                elif args.fault in ("takeover", "reselection"):
                    with (output / "takeover.log").open("w") as stream:
                        takeover = subprocess.run([str(px4 / "build/px4_sitl_default/bin/px4-commander"),
                            "--instance", "0", "land"], stdout=stream, stderr=subprocess.STDOUT, timeout=5)
                    if takeover.returncode:
                        raise RuntimeError("Native operator Land takeover failed")
                    if args.fault == "reselection":
                        interrupted = wait_for("operator_takeover_before_reselection", lambda snap:
                            status(snap).get("phase") in ("failed", "interrupted") and not status(snap).get("active"), 4)
                        operator_select(status(interrupted)["mode_id"])
                elif args.fault == "process-death":
                    os.killpg(mission_process.pid, signal.SIGKILL)
                    mission_process.wait(timeout=5)
                elif args.fault == "active-edit":
                    candidate = json.loads((output / "mission.json").read_text())
                    candidate["mission"]["items"].insert(-1, {"type": "localHold", "duration_s": 1.0})
                    atomic_json(output / "mission.json", candidate)
                injected = True
                result["fault_injected"] = True
                event("fault_injected", fault=args.fault, action=args.fault_action)
            if injected and args.fault not in ("none", "active-edit"):
                native_disarmed = message(snapshot, "vehicle_status").get("arming_state") == 1
                native_landed = message(snapshot, "vehicle_land_detected").get("landed") is True
                if native_disarmed and native_landed:
                    atomic_json(output / "fault_recovery.json", snapshot)
                    break
                time.sleep(.1)
                continue
            if state.get("terminal"):
                atomic_json(output / "terminal.json", snapshot)
                if not state.get("success"):
                    raise RuntimeError("Mission terminated: " + str(state))
                if state.get("armed") or not state.get("landed"):
                    raise RuntimeError("Success without fresh landed/disarmed state")
                break
            time.sleep(.1)
        else:
            raise RuntimeError("Mission flight exceeded bounded deadline")
        time.sleep(5)
        final = latest()
        expected_failure = injected and args.fault not in ("none", "active-edit")
        if not expected_failure and (not status(final).get("success") or status(final).get("armed")):
            raise RuntimeError("Terminal state did not remain landed/disarmed")
        if expected_failure and (message(final, "vehicle_status").get("arming_state") != 1 or
                message(final, "vehicle_land_detected").get("landed") is not True):
            raise RuntimeError("Fault episode did not remain landed/disarmed")
        if args.fault != "none" and not injected:
            raise RuntimeError("Requested fault action was never observed")
        if args.screenshot and not captured:
            raise RuntimeError("No actual airborne GUI screenshot was captured")
        if args.fault == "active-edit":
            if status(final).get("revision") != revision or not status(final).get("update_error"):
                raise RuntimeError("Active file edit was applied or its rejection was lost")
            atomic_json(output / "active_edit_rejection.json", final)
            result["completed_status"] = status(final)
            # A new explicit write after landing is required. The previously
            # rejected edit must not be deferred, and auto_start is consumed.
            candidate = json.loads((output / "mission.json").read_text())
            atomic_json(output / "mission.json", candidate)
            wait_for("postflight_reload", lambda snap: status(snap).get("revision") != revision and
                     not status(snap).get("update_error") and not status(snap).get("terminal"), 15)
            time.sleep(5)
            final = latest()
            if status(final).get("armed") or status(final).get("started") or not status(final).get("landed"):
                raise RuntimeError("Inactive reload resumed or rearmed the consumed auto_start episode")
            atomic_json(output / "postflight_no_autostart.json", final)
        atomic_json(output / "final.json", final)
        shutil.copy2(output / "collector/graph_latest.json", output / "flight_graph.json")
        result.update(success=True, final_status=status(final))
        event("fault_recovered_landed_disarmed" if expected_failure else "landed_disarmed_success")
    except BaseException as exception:
        result["error"] = repr(exception)
        atomic_json(output / "failure_snapshot.json", latest())
        event("episode_failed", error=repr(exception))
        if message(latest(), "vehicle_status").get("arming_state") == 2:
            try:
                wait_for("failure_landed_disarmed", lambda snap:
                    message(snap, "vehicle_status").get("arming_state") == 1 and
                    message(snap, "vehicle_land_detected").get("landed") is True, 120)
            except BaseException as recovery_error:
                result["recovery_error"] = repr(recovery_error)
    finally:
        for pid in paused:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        shutdown = []
        for name, process, stream in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            shutdown.append(dict(name=name, pid=process.pid, exit_code=process.returncode))
            stream.close()
        log = (output / "base.log").read_text(errors="replace") if (output / "base.log").exists() else ""
        paths = re.findall(r"\./log/(\d{4}-\d{2}-\d{2}/[\d_]+\.ulg)", log)
        if paths:
            source = px4 / "build/px4_sitl_default/rootfs/0/log" / paths[-1]
            if source.exists():
                shutil.copy2(source, output / "flight.ulg")
                result["ulog_sha256"] = hashlib.sha256((output / "flight.ulg").read_bytes()).hexdigest()
        result.update(ended_utc=utc_now(), shutdown=shutdown)
        atomic_json(output / "result.json", result)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
