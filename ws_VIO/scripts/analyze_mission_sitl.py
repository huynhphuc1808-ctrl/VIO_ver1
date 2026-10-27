#!/usr/bin/env python3
"""Read-only analysis of one JSON mission episode, collector and native ULog."""

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re

from collect_phase1_sitl import atomic_json, utc_now


def analyze(run):
    import numpy as np
    from pyulog import ULog

    result = json.loads((run / "result.json").read_text())
    definition = run / "definition_before_run.json"
    fixture = json.loads((definition if definition.exists() else run / "mission.json").read_text())
    checks, metrics = {}, {}

    def check(name, passed, observed):
        checks[name] = {"pass": bool(passed), "observed": observed}

    rows = defaultdict(list)
    with (run / "collector/messages.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            key = re.sub(r"_v\d+$", "", row["topic"].rsplit("/", 1)[-1])
            if row["topic"] == "/uav_vio/mission/status":
                key = "runtime"
                row["message"] = row.get("parsed_json") or json.loads(row["message"]["data"])
            if key in ("runtime", "vehicle_status", "vehicle_local_position", "vehicle_command",
                       "vehicle_command_mode_executor", "goto_setpoint") or row["topic"] == "/vio/odometry":
                rows[key].append(row)
    runtime = rows["runtime"]
    ids = runtime[-1]["message"] if runtime else {}
    executor, local_mode, owned_mode = (ids.get(k, -1) for k in ("executor_id", "local_mode_id", "mode_id"))
    source = 1000 + executor
    bootstrap_source = result.get("bootstrap_source_component", 1)
    completed = next((r["message"] for r in reversed(runtime) if r["message"].get("phase") == "completed"), {})
    check("runtime_completed", result.get("success") and completed.get("success") and completed.get("terminal")
          and completed.get("landed") and not completed.get("armed"), completed)
    if result.get("fault") == "active-edit":
        rejected = json.loads((run / "active_edit_rejection.json").read_text())["topics"]["/uav_vio/mission/status"]["parsed_json"]
        reloaded = json.loads((run / "postflight_no_autostart.json").read_text())["topics"]["/uav_vio/mission/status"]["parsed_json"]
        check("active_edit_not_queued_and_explicit_inactive_reload_no_rearm", bool(rejected.get("update_error")) and
              rejected.get("revision") == completed.get("revision") and
              reloaded.get("revision") != completed.get("revision") and not reloaded.get("armed") and
              not reloaded.get("started") and reloaded.get("landed") and not reloaded.get("terminal"), reloaded)
    graph = json.loads((run / "flight_graph.json").read_text())
    publishers = {topic: entry.get("publishers", []) for topic, entry in graph["monitored"].items()
                  if "/in/vehicle_command" in topic or "/in/goto_setpoint" in topic}
    check("single_command_backend", sum(n["name"] == "uav_vio_mission" for n in graph["nodes"]) == 1
          and any(publishers.values()) and all(p["node_name"] == "uav_vio_mission"
              for entries in publishers.values() for p in entries), publishers)
    expected = [item["type"] for item in fixture["mission"]["items"]]
    observed = {(r["message"].get("index"), r["message"].get("action")) for r in runtime}
    check("all_actions_observed", all((i, name) in observed for i, name in enumerate(expected)),
          {"expected": expected, "observed": sorted(observed, key=str)})
    check("no_runtime_failure", not any(r["message"].get("phase") in ("failed", "interrupted") for r in runtime),
          [r["message"] for r in runtime if r["message"].get("error")])
    path = run / "flight.ulg"
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    check("ulog_integrity", result.get("ulog_sha256") == sha, sha)
    ulog = ULog(str(path))
    data = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
    status, local, land, flags, commands = (data[k] for k in
        ("vehicle_status", "vehicle_local_position", "vehicle_land_detected", "estimator_status_flags", "vehicle_command"))
    armed = np.flatnonzero(status["arming_state"] == 2)
    check("native_airborne_landed_disarmed", len(armed) > 0 and np.any(land["landed"] == 0)
          and status["arming_state"][-1] == 1 and bool(land["landed"][-1]),
          {"arming": sorted(set(int(v) for v in status["arming_state"])),
           "final_landed": bool(land["landed"][-1])})
    if not len(armed):
        raise RuntimeError("No armed native ULog interval")
    start, end = int(status["timestamp"][armed[0]]), int(status["timestamp"][armed[-1]])
    mask = (local["timestamp"] >= start) & (local["timestamp"] <= end)
    check("local_feedback_valid", all(np.all(local[k][mask]) for k in
          ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")), int(np.sum(mask)))
    check("no_native_failsafe", not np.any(status["failsafe"][armed]),
          sorted(set(bool(v) for v in status["failsafe"][armed])))
    params = ulog.initial_parameters
    check("gnss_disabled", all(params.get(k) == 0 for k in ("SYS_HAS_GPS", "SIM_GZ_EN_GPS", "EKF2_GPS_CTRL")),
          {k: params.get(k) for k in ("SYS_HAS_GPS", "SIM_GZ_EN_GPS", "EKF2_GPS_CTRL", "EKF2_EV_CTRL")})
    fmask = (flags["timestamp"] >= start) & (flags["timestamp"] <= end)
    fractions = {k: float(np.mean(flags[k][fmask])) for k in
                 ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt", "cs_gnss_pos", "cs_gnss_vel")}
    check("continuous_native_ev_without_gnss", all(fractions[k] == 1 for k in
          ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt")) and
          all(fractions[k] == 0 for k in ("cs_gnss_pos", "cs_gnss_vel")), fractions)
    command_summary = []
    for i, command in enumerate(commands["command"]):
        if int(command) not in (400, 22, 34, 100001):
            continue
        first = float(commands["param1"][i])
        command_summary.append({"command": int(command), "param1": first if math.isfinite(first) else None,
                                "source": int(commands["source_component"][i])})
    needed = [(400, 1), (22, None), (100001, local_mode), (100001, 18)]
    source_ok = all(any(c["command"] == cmd and c["param1"] == param and c["source"] == source
                        for c in command_summary) for cmd, param in needed)
    source_ok = source_ok and all(c["source"] == (bootstrap_source if c["command"] == 100001 and
          c["param1"] == owned_mode else source) for c in command_summary)
    check("same_executor_commands", source_ok, command_summary)
    bootstrap = [i for i, command in enumerate(commands["command"]) if int(command) == 100001 and
                 float(commands["param1"][i]) == owned_mode and int(commands["source_component"][i]) == bootstrap_source]
    arm_commands = [i for i, command in enumerate(commands["command"]) if int(command) == 400 and
                    float(commands["param1"][i]) == 1]
    ownership = []
    if len(bootstrap) == 1 and len(arm_commands) == 1:
        select_time = int(commands["timestamp"][bootstrap[0]])
        arm_time = int(commands["timestamp"][arm_commands[0]])
        ownership = [int(status["timestamp"][i]) for i in range(len(status["timestamp"]))
            if select_time <= int(status["timestamp"][i]) < arm_time and status["arming_state"][i] == 1 and
            status["executor_in_charge"][i] == executor and status["nav_state_user_intention"][i] == owned_mode]
    check("native_disarmed_ownership_before_arm", bool(ownership), ownership)
    ros_status = rows["vehicle_status"]
    armed_rows = [r for r in ros_status if r["message"]["arming_state"] == 2]
    after = [r for r in ros_status if armed_rows and r["receipt_steady_s"] > armed_rows[-1]["receipt_steady_s"]
             and r["message"]["arming_state"] == 1]
    receipt_metrics = {}
    if armed_rows and after:
        a, b = armed_rows[0]["receipt_steady_s"], after[0]["receipt_steady_s"]
        for key, limit in (("vehicle_status", 2), ("vehicle_local_position", .5), ("odometry", .5)):
            times = [r["receipt_steady_s"] for r in rows[key] if a <= r["receipt_steady_s"] <= b]
            receipt_metrics[key] = {"max_gap_s": max(np.diff([a, *times, b])), "limit_s": limit}
    check("flight_receipt_freshness", len(receipt_metrics) == 3 and all(
        m["max_gap_s"] < m["limit_s"] for m in receipt_metrics.values()), receipt_metrics)
    frozen = next((r for r in runtime if r["message"].get("frame_valid") and
                   not r["message"].get("frame_initializing")), None)
    check("mission_frame_frozen", bool(frozen), frozen["message"] if frozen else None)
    if frozen:
        origin = frozen["message"]["origin_ned"]
        height = float(origin[2] - np.min(local["z"][mask]))
        target = fixture["mission"]["items"][0]["altitude_m"]
        check("configured_takeoff_height", height >= target - .5, {"max_height_m": height, "target_m": target})
        truth = data.get("vehicle_local_position_groundtruth")
        if truth is None:
            check("physical_height_from_evaluation_only_groundtruth", False, "groundtruth dataset missing")
        else:
            gtmask = (truth["timestamp"] >= start) & (truth["timestamp"] <= end)
            before_arm = np.flatnonzero(truth["timestamp"] <= start)
            ground_z = float(truth["z"][before_arm[-1]]) if len(before_arm) else float(truth["z"][0])
            physical_height = float(ground_z - np.min(truth["z"][gtmask]))
            check("physical_height_from_evaluation_only_groundtruth", target - .5 <= physical_height <= target + .75,
                  {"max_height_m": physical_height, "target_m": target,
                   "boundary": "Read only after flight; not an estimator/controller input."})
    capture_path = run / "airborne_snapshot.json"
    if capture_path.exists():
        capture = json.loads(capture_path.read_text())
        image = (run / "airborne.png").read_bytes()
        observations = []
        for snapshot in (capture["before"], capture["after"]):
            state = snapshot["topics"]["/uav_vio/mission/status"]["parsed_json"]
            local_row = next(row for topic, row in snapshot["topics"].items()
                             if re.search(r"/vehicle_local_position(?:_v\d+)?$", topic))
            timestamp = local_row["message"]["timestamp"]
            truth = data["vehicle_local_position_groundtruth"]
            i = int(np.argmin(np.abs(truth["timestamp"].astype(float) - timestamp)))
            initial_z = float(truth["z"][0])
            observations.append({"armed_hold": state.get("armed") and not state.get("landed") and
                                 state.get("action") == "localHold" and state.get("command_owner"),
                                 "native_timestamp_us": timestamp,
                                 "local_age_s": snapshot["updated_steady_s"] - local_row["receipt_steady_s"],
                                 "physical_height_m": initial_z - float(truth["z"][i])})
        check("same_episode_actual_airborne_gui_capture", image.startswith(b"\x89PNG\r\n\x1a\n") and
              hashlib.sha256(image).hexdigest() == capture["image_sha256"] and
              0 <= capture["after"]["updated_steady_s"] - capture["before"]["updated_steady_s"] <= 3 and
              all(o["armed_hold"] and o["local_age_s"] < .5 and 1.5 <= o["physical_height_m"] <= 2.75
                  and start <= o["native_timestamp_us"] <= end for o in observations), observations)
    metrics.update(armed_start_us=start, armed_end_us=end, dynamic_ids={
        "executor_id": executor, "local_mode_id": local_mode, "owned_mode_id": owned_mode})
    return {"schema": "json-mission-sitl-analysis-v1", "run": str(run), "analyzed_utc": utc_now(),
            "pass": all(c["pass"] for c in checks.values()), "checks": checks, "metrics": metrics,
            "limitations": ["This validates one nominal episode, not the stale/takeover/lifecycle fault matrix.",
                            "Orbit radius/speed/direction/laps require the separate circle analyzer.",
                            "No hardware flight is certified; GUI capture is checked only when supplied."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = analyze(args.run.resolve())
    except Exception as error:
        report = {"pass": False, "error": repr(error), "run": str(args.run.resolve())}
    atomic_json(args.output, report)
    print(json.dumps({"pass": report["pass"], "failed_checks": [k for k, v in report.get("checks", {}).items()
                    if not v["pass"]], "error": report.get("error")}))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
