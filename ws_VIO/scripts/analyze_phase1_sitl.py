#!/usr/bin/env python3
"""Read-only Phase 1 and minimal-demo evidence analysis; no flight commands."""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re
import sys

from collect_phase1_sitl import atomic_json, utc_now


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def decode_name(value):
    return (value if isinstance(value, str) else bytes(value).decode("utf-8", "replace")).rstrip("\0")


def analyze(run):
    import numpy as np
    from pyulog import ULog

    result = {"schema": "phase1-minimal-demo-analysis-v1", "run": str(run),
              "analyzed_utc": utc_now(), "phase1": {}, "demo": {}, "evidence": {},
              "limitations": ["Scope is Phase 1 plus a minimal demo; full Phase 2 is not accepted.",
                  "ROS receipt ordering between topics can differ by 0.15 wall seconds at mode transitions.",
                  "The demo requires at least 2 ROS seconds of sampled Hold; configured extra screenshot dwell is not exact timing certification.",
                  "PNG integrity and synchronized telemetry require separate visual inspection of the screenshot."]}
    evidence = result["evidence"]

    def check(section, name, passed, observed=None):
        result[section][name] = {"pass": None if passed is None else bool(passed), "observed": observed}

    events_path = run / "events.jsonl"
    events = [json.loads(line) for line in events_path.read_text().splitlines()] if events_path.exists() else []
    starts = [e for e in events if e.get("event") == "process_started" and e.get("name") == "mission_flight"]
    flight_start = starts[-1]["steady_s"] if starts else float("inf")
    evidence["flight_process_event"] = starts[-1] if starts else None
    parent = run.parent if (run / "parent_evidence.json").exists() else run
    for name in ("build", "ctest", "compatibility"):
        p = next((p for p in (run / f"{name}.exit_code", parent / f"{name}.exit_code") if p.exists()), None)
        check("phase1", name, int(p.read_text()) == 0 if p else None, str(p) if p else "missing exit code")

    records = defaultdict(list)
    aliases = {"vehicle_status": "status", "vehicle_local_position": "local",
        "vehicle_land_detected": "land", "vehicle_command": "cmd", "vehicle_command_mode_executor": "executor_cmd",
        "goto_setpoint": "goto", "register_ext_component_reply": "registration", "arming_check_reply": "gate",
        "estimator_status_flags": "flags", "vehicle_visual_odometry": "ev"}
    clock = None
    path = run / "collector/messages.jsonl"
    if path.exists():
        with path.open() as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                except ValueError:
                    check("phase1", "collector_jsonl_integrity", False, {"line": line_no})
                    continue
                topic, msg, wall = row["topic"], row["message"], row["receipt_steady_s"]
                if topic == "/clock":
                    clock = msg["clock"]["sec"] + msg["clock"]["nanosec"] * 1e-9
                    continue
                key = ({"/uav_vio/mission/status": "runtime", "/vio/odometry": "vio",
                        "/vio/status": "vio_status"}.get(topic) or
                       aliases.get(re.sub(r"_v\d+$", "", topic.rsplit("/", 1)[-1])))
                if key:
                    if key == "runtime":
                        msg = row.get("parsed_json") or json.loads(msg["data"])
                    if key in ("vio", "ev"):
                        msg = {"timestamp": msg.get("timestamp")}
                    records[key].append({"wall": wall, "ros": clock, "m": msg})
        result["phase1"].setdefault("collector_jsonl_integrity", {"pass": True, "observed": str(path)})
    else:
        check("phase1", "collector_jsonl_integrity", None, "messages.jsonl missing")

    def before(key, wall):
        return next((r for r in reversed(records[key]) if r["wall"] <= wall), None)

    def nearest(key, wall):
        return min(records[key], key=lambda r: abs(r["wall"] - wall)) if records[key] else None

    def regs_at(wall):
        return {decode_name(r["m"]["name"]): r["m"] for r in records["registration"]
                if r["wall"] <= wall and r["m"].get("success")}

    snap = read_json(run / "registration_no_global_snapshot.json", {})
    topics = snap.get("topics", {})
    ground_runtime = topics.get("/uav_vio/mission/status", {}).get("parsed_json", {})
    local = next((v["message"] for k, v in topics.items() if "/vehicle_local_position" in k), {})
    ground_regs = regs_at(snap.get("updated_steady_s", 0))
    gate = ground_regs.get("VIO Mission Gate", {})
    check("phase1", "registered_without_global_reference", bool(ground_runtime.get("registered")) and
        local.get("xy_global") is False and local.get("z_global") is False and local.get("ref_timestamp") == 0 and
        ground_regs.get("VIO Mission", {}).get("mode_id") == ground_runtime.get("mode_id") and
        ground_regs.get("VIO Local Goto", {}).get("mode_id") == ground_runtime.get("local_mode_id") and
        gate.get("mode_id") == -1 and gate.get("mode_executor_id") == -1 if snap else None,
        {"runtime": ground_runtime, "local_reference": {k: local.get(k) for k in ("xy_global", "z_global", "ref_timestamp")},
         "registrations": ground_regs})
    parent_evidence = read_json(run / "parent_evidence.json", {})
    reference = parent_evidence.get("ground_fault_evidence_run")
    evidence["explicit_ground_fault_evidence_run"] = reference
    for name in ("prearm_no_vio", "prearm_stale_vio", "prearm_stale_local"):
        source_run, source_gate = run, gate
        registration_path = run / "registration_no_global_snapshot.json"
        source_registration_valid = bool(snap) and bool(ground_runtime.get("registered"))
        if not (run / f"{name}.json").exists() and name in ("prearm_stale_vio", "prearm_stale_local") and reference:
            reference_path = Path(reference)
            # Cross-run proof is allowed only by an explicit absolute reference;
            # never search earlier attempts for a convenient passing artifact.
            if reference_path.is_absolute():
                source_run = reference_path.resolve()
                registration_path = source_run / "registration_no_global_snapshot.json"
                reference_snapshot = read_json(registration_path, {})
                reference_topics = reference_snapshot.get("topics", {})
                source_gate = next((v.get("message", {}) for k, v in reference_topics.items()
                    if "/register_ext_component_reply" in k and
                    decode_name(v.get("message", {}).get("name", [])) == "VIO Mission Gate"), {})
                reference_runtime = reference_topics.get("/uav_vio/mission/status", {}).get("parsed_json", {})
                reference_local = next((v.get("message", {}) for k, v in reference_topics.items()
                                        if "/vehicle_local_position" in k), {})
                source_registration_valid = (bool(reference_snapshot) and reference_runtime.get("registered") is True and
                    reference_local.get("xy_global") is False and reference_local.get("z_global") is False and
                    reference_local.get("ref_timestamp") == 0)
        probe_path = source_run / f"{name}.json"
        probe = read_json(probe_path, {})
        ack = probe.get("rejection_ack", {}).get("message", {})
        commands = probe.get("commands", [])
        prereqs = probe.get("prerequisites", {})
        reader = probe.get("command_readers_pre_send", {})
        check("phase1", name, probe.get("exit_code") == 0 and ack.get("command") == 400 and
            ack.get("result") in (1, 2) and ack.get("target_component") == 191 and
            source_registration_valid and source_gate.get("success") is True and source_gate.get("mode_id") == -1 and
            source_gate.get("mode_executor_id") == -1 and probe.get("gate_id") == source_gate.get("arming_check_id") and
            bool(prereqs) and all(prereqs.values()) and
            len(commands) == 1 and commands[0]["message"].get("param2") == 0 and
            commands[0]["message"].get("param1") == 1 and probe.get("observed_disarmed_after_ack_wall_s", 0) >= 3 and
            not probe.get("unexpected_arm_observed", True) if probe else None,
            {"exit_code": probe.get("exit_code"), "ack": ack, "gate_id": probe.get("gate_id"),
             "prerequisites": prereqs, "matched_command_readers": reader.get("matched_subscription_count"),
             "evidence_path": str(probe_path), "source_run": str(source_run),
             "cross_run_ground_proof": source_run != run, "registration_snapshot_path": str(registration_path),
             "registration_snapshot_valid": source_registration_valid, "registered_global_gate": source_gate})

    invalid_events = [e for e in events if e.get("event") == "command_finished" and e.get("name") in
                      ("invalid_empty", "invalid_malformed", "invalid_nodes")]
    invalid_end = next((e["steady_s"] for e in invalid_events if e["name"] == "invalid_nodes"), 0)
    invalid_start = next((e["steady_s"] for e in events if e.get("event") == "native_telemetry_available"), invalid_end)
    invalid = {}
    for name in ("invalid_empty", "invalid_malformed"):
        code, log = run / f"{name}.exit_code", run / f"{name}.log"
        invalid[name] = code.exists() and int(code.read_text()) == 1 and log.exists() and "startup/runtime error" in log.read_text()
    nodes = run / "invalid_nodes.log"
    source = Path(__file__).resolve().parents[1] / "src/uav_vio_mission/src/mission_runtime.cpp"
    text = source.read_text()
    invalid["parse_before_endpoints_source"] = text.index("definition =") < text.index("context =")
    invalid["no_runtime_node_after_rejection"] = nodes.exists() and not re.search(
        r"^/(?:uav_vio_mission|uav_vio_circle_mission)$", nodes.read_text(), re.M)
    invalid["no_command_endpoints_emitted"] = invalid_end > invalid_start and not any(
        invalid_start <= r["wall"] <= invalid_end for key in ("cmd", "executor_cmd", "goto", "registration") for r in records[key])
    check("phase1", "invalid_intent_rejected_before_endpoints", all(invalid.values()), invalid)
    evidence["runtime_source_sha256"] = digest(source)

    runtime = [r for r in records["runtime"] if r["wall"] >= flight_start]
    registered = next((r for r in runtime if r["m"].get("registered")), None)
    ids = {k: registered["m"].get(k) for k in ("mode_id", "local_mode_id", "executor_id", "revision")} if registered else {}
    evidence["flight_ids"] = ids
    mode, local_mode, executor = ids.get("mode_id"), ids.get("local_mode_id"), ids.get("executor_id")
    component = 1000 + executor if isinstance(executor, int) and executor >= 0 else None
    flight_regs = {decode_name(r["m"]["name"]): r["m"] for r in records["registration"] if r["wall"] >= flight_start}
    evidence["flight_registrations"] = flight_regs
    check("phase1", "fresh_flight_registration", registered is not None and
        flight_regs.get("VIO Mission", {}).get("mode_id") == mode and
        flight_regs.get("VIO Mission", {}).get("mode_executor_id") == executor and
        flight_regs.get("VIO Local Goto", {}).get("mode_id") == local_mode and
        flight_regs.get("VIO Mission Gate", {}).get("success") is True)
    stopped = [e for e in events if e.get("event") == "ground_runtime_and_fault_relay_stopped"]
    check("phase1", "single_sequential_flight_episode", len(starts) == 1 and bool(stopped) and
          stopped[-1]["steady_s"] < flight_start and any("auto_start:=true" in arg for arg in starts[-1].get("args", [])) if starts else None)
    commands = sorted([r for key in ("cmd", "executor_cmd") for r in records[key] if r["wall"] >= flight_start], key=lambda r: r["wall"])
    specs = [("bootstrap", 100001, mode, 1), ("executor_arm", 400, 1, component),
             ("native_takeoff", 22, None, component), ("local_handoff", 100001, local_mode, component),
             ("native_land", 100001, 18, component)]
    sequence, cursor = {}, flight_start
    for name, command, param1, sender in specs:
        found = next((r for r in commands if r["wall"] >= cursor and r["m"].get("command") == command and
            r["m"].get("source_component") == sender and (param1 is None or r["m"].get("param1") == param1)), None)
        sequence[name] = found
        if found:
            cursor = found["wall"]
    check("phase1", "owned_command_sequence", all(sequence.values()), sequence)
    arm = sequence.get("executor_arm")
    ownership = [r for r in records["status"] if arm and flight_start <= r["wall"] <= arm["wall"] + .15 and
                 r["m"].get("arming_state") == 1 and r["m"].get("executor_in_charge") == executor and
                 r["m"].get("nav_state_user_intention") == mode]
    strictly_before = [r for r in ownership if r["wall"] <= arm["wall"]] if arm else []
    chosen = strictly_before[-1] if strictly_before else ownership[0] if ownership else None
    check("phase1", "raw_disarmed_executor_ownership_before_arm", bool(chosen) if arm else None,
          {"status": chosen, "receipt_reordering_used": bool(chosen and not strictly_before),
           "receipt_offset_from_arm_s": chosen["wall"]-arm["wall"] if chosen else None})
    fresh = {}
    if arm:
        for key, limit in (("status", 2), ("local", .5), ("vio", .5), ("runtime", .5), ("flags", 2)):
            r = before(key, arm["wall"])
            fresh[key] = {"age_wall_s": arm["wall"] - r["wall"] if r else None, "message": r["m"] if r else None}
            fresh[key]["fresh"] = r is not None and arm["wall"] - r["wall"] <= limit
        l = fresh["local"]["message"] or {}
        ready = any(r["m"].get("ready") and r["wall"] <= arm["wall"] and arm["wall"] - r["wall"] <= 2 for r in runtime)
        check("phase1", "new_runtime_fresh_positive_prearm", all(v["fresh"] for v in fresh.values()) and ready and
              all(l.get(k) is True for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")), fresh)
    else:
        check("phase1", "new_runtime_fresh_positive_prearm", None)
    graph = read_json(run / "flight_graph.json", {})
    endpoints = {k: v.get("publishers", []) for k, v in graph.get("monitored", {}).items()
                 if any(name in k for name in ("vehicle_command", "goto_setpoint")) and "/in/" in k}
    publishers = [v for entries in endpoints.values() if isinstance(entries, list) for v in entries]
    check("phase1", "one_flight_backend_graph", bool(publishers) and all(p.get("node_name") == "uav_vio_mission" for p in publishers)
        and sum(n.get("name") == "uav_vio_mission" for n in graph.get("nodes", [])) == 1 and
        not any(n.get("name") == "uav_vio_circle_mission" for n in graph.get("nodes", [])) if graph else None, endpoints)

    exceptions, bad = [], []
    gotos = [r for r in records["goto"] if r["wall"] >= flight_start]
    for r in gotos:
        status, state = nearest("status", r["wall"]), nearest("runtime", r["wall"])
        status_ok = status and abs(status["wall"]-r["wall"]) <= 2 and status["m"].get("arming_state") == 2 and status["m"].get("nav_state") == local_mode
        active_ok = state and abs(state["wall"]-r["wall"]) <= .5 and state["m"].get("active") is True
        if not status_ok or not active_ok:
            nearby_status = any(abs(s["wall"]-r["wall"]) <= .15 and s["m"].get("arming_state") == 2 and s["m"].get("nav_state") == local_mode for s in records["status"])
            nearby_active = active_ok or any(abs(s["wall"]-r["wall"]) <= .15 and s["m"].get("active") is True for s in runtime)
            (exceptions if nearby_status and nearby_active else bad).append({"setpoint_wall_s": r["wall"], "nearest_status": status})
    check("demo", "goto_only_armed_active_local_mode", bool(gotos) and not bad,
          {"count": len(gotos), "transition_receipt_exceptions": exceptions, "violations": bad})
    terminal = runtime[-1]["m"] if runtime else {}
    check("demo", "runtime_terminal_success", terminal.get("phase") == "completed" and terminal.get("success") is True and
          terminal.get("terminal") is True and terminal.get("landed") is True and terminal.get("armed") is False if runtime else None, terminal)
    completed = next((r for r in runtime if r["m"].get("success") and r["m"].get("terminal")), None)
    check("demo", "no_failure_revision_change_or_rearm", bool(completed) and
          all(r["m"].get("phase") not in ("failed", "interrupted") for r in runtime) and
          all(r["m"].get("revision") == ids.get("revision") for r in runtime if r["m"].get("registered")) and
          not any(r["wall"] > completed["wall"] and r["m"].get("command") == 400 and r["m"].get("param1") == 1 for r in commands)
          if runtime else None)
    hold = [r for r in runtime if r["m"].get("action") == "localHold" and r["m"].get("active") and r["m"].get("armed")]
    fixture = read_json(run / "demo_mission.json", {})
    configured_hold = sum(i.get("duration_s", 0) for i in fixture.get("mission", {}).get("items", []) if i.get("type") == "localHold")
    minimum_demo_hold = 2.0
    if hold and all(r["ros"] is not None for r in hold):
        after = next((r for r in runtime if r["wall"] > hold[-1]["wall"] and r["ros"] is not None), None)
        earlier = before("runtime", hold[0]["wall"] - 1e-9)
        span = hold[-1]["ros"] - hold[0]["ros"]
        possible = after["ros"] - earlier["ros"] if after and earlier and earlier["ros"] is not None else None
        check("demo", "hold_duration_ros_s", True if configured_hold >= minimum_demo_hold and span >= minimum_demo_hold else
              False if possible is not None and possible < minimum_demo_hold else None,
              {"demo_minimum_hold_s": minimum_demo_hold, "configured_hold_s": configured_hold,
               "observed_lower_bound": span, "possible_upper_bound": possible,
               "exact_configured_dwell_certified": span >= configured_hold if configured_hold else None})
    else:
        check("demo", "hold_duration_ros_s", None, "hold or /clock data absent")

    ulog_path = run / "flight.ulg"
    if ulog_path.exists():
        ulog = ULog(str(ulog_path))
        evidence["ulog"] = {"path": str(ulog_path), "sha256": digest(ulog_path)}
        data = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
        vs, ld, lp, flags = (data.get(k, {}) for k in ("vehicle_status", "vehicle_land_detected", "vehicle_local_position", "estimator_status_flags"))
        native_start = before("local", flight_start)
        epoch = native_start["m"]["timestamp"] if native_start else 0
        armed = np.where((vs.get("timestamp", np.array([])) >= epoch) & (vs.get("arming_state", np.array([])) == 2))[0]
        if len(armed):
            first, last = int(vs["timestamp"][armed[0]]), int(vs["timestamp"][armed[-1]])
            disarmed = np.where((vs["timestamp"] > last) & (vs["arming_state"] == 1))[0]
            end = int(vs["timestamp"][disarmed[0]]) if len(disarmed) else last
            airborne = (ld["timestamp"] >= first) & (ld["timestamp"] <= end) & (ld["landed"] == 0) if ld else np.array([])
            air_times = ld["timestamp"][airborne] if ld else []
            check("demo", "ulog_armed_airborne_final_disarmed_landed", len(air_times) > 1 and
                  (int(air_times[-1])-int(air_times[0])) >= 2000000 and len(disarmed) > 0 and
                  int(vs["arming_state"][-1]) == 1 and bool(ld["landed"][-1]),
                  {"armed_start_us": first, "armed_end_us": end, "airborne_first_us": int(air_times[0]) if len(air_times) else None,
                   "airborne_last_us": int(air_times[-1]) if len(air_times) else None})
            native_modes = list(dict.fromkeys(int(v) for v in vs["nav_state"][(vs["timestamp"] >= first) & (vs["timestamp"] <= end)]))
            check("demo", "ulog_native_modes", all(m in native_modes for m in (17, local_mode, 18)), native_modes)
            frozen = next((r for r in runtime if r["m"].get("frame_valid") and not r["m"].get("frame_initializing") and r["m"].get("origin_ned")), None)
            window = (lp["timestamp"] >= first) & (lp["timestamp"] <= end) if lp else np.array([])
            height = float(frozen["m"]["origin_ned"][2] - np.min(lp["z"][window])) if frozen and np.any(window) else None
            check("demo", "height_from_frozen_origin", height >= .75 if height is not None else None,
                  {"max_height_m": height, "frozen_origin": frozen["m"].get("origin_ned") if frozen else None})
            mask = (flags["timestamp"] >= first) & (flags["timestamp"] <= end) if flags else np.array([])
            fractions = {k: float(np.mean(flags[k][mask])) for k in ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt", "cs_gnss_pos", "cs_gnss_vel", "cs_gps_hgt") if k in flags and np.any(mask)}
            check("demo", "ev_fusion_without_gnss", len(fractions) == 6 and all(fractions[k] >= .5 for k in ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt")) and
                  all(fractions[k] == 0 for k in ("cs_gnss_pos", "cs_gnss_vel", "cs_gps_hgt")) if fractions else None, fractions)
            for kind in ("pos", "vel", "hgt"):
                aid = data.get(f"estimator_aid_src_ev_{kind}", {})
                aid_mask = (aid["timestamp"] >= first) & (aid["timestamp"] <= end) if aid else np.array([])
                if aid and np.any(aid_mask) and "time_last_fuse" in aid and "fused" in aid:
                    advance = int(np.max(aid["time_last_fuse"][aid_mask])) - int(np.min(aid["time_last_fuse"][aid_mask]))
                    fused = float(np.mean(aid["fused"][aid_mask]))
                    check("demo", f"ev_aid_{kind}", fused > .01 and advance > 1000000, {"fused_fraction": fused, "last_fuse_advance_us": advance})
                else:
                    log = run / f"airborne_native_estimator_aid_src_ev_{kind}.log"
                    text = log.read_text() if log.exists() else ""
                    check("demo", f"ev_aid_{kind}", bool(re.search(r"fused:\s*True", text)) if text else None, {"fallback_native_log": str(log), "advancement": "not logged"})
            flight_status = [r for r in records["status"] if r["wall"] >= flight_start and r["m"].get("arming_state") == 2]
            wall_start = flight_status[0]["wall"] if flight_status else None
            wall_end = next((r["wall"] for r in records["status"] if flight_status and r["wall"] > flight_status[-1]["wall"] and r["m"].get("arming_state") == 1), None)
            gaps = {}
            if wall_start is not None and wall_end is not None:
                for key, limit in (("status", 2), ("local", .5), ("vio", .5)):
                    times = [r["wall"] for r in records[key] if wall_start <= r["wall"] <= wall_end]
                    maximum = float(np.max(np.diff([wall_start, *times, wall_end])))
                    gaps[key] = {"count": len(times), "rate_wall_hz": len(times)/(wall_end-wall_start), "max_gap_wall_s": maximum, "limit_s": limit}
                check("demo", "flight_receipt_freshness", all(v["count"] > 1 and v["max_gap_wall_s"] < v["limit_s"] for v in gaps.values()), gaps)
            else:
                check("demo", "flight_receipt_freshness", None)
        else:
            check("demo", "ulog_armed_airborne_final_disarmed_landed", False, "No armed ULog interval after flight process start")
    else:
        check("demo", "ulog_physical_proof", None, "flight.ulg missing")

    capture = read_json(run / "airborne_snapshot.json", {})
    screenshot = run / "airborne.png"
    screenshot_states = []
    for side in ("before", "after"):
        snapshot = capture.get(side, {})
        items = snapshot.get("topics", {})
        by_key = {key: next((v for k, v in items.items() if token in k), {}) for key, token in
                  (("runtime", "/uav_vio/mission/status"), ("status", "/vehicle_status"), ("land", "/vehicle_land_detected"),
                   ("local", "/vehicle_local_position"), ("vio", "/vio/odometry"))}
        state = by_key["runtime"].get("parsed_json", {})
        okay = state.get("action") == "localHold" and state.get("armed") is True and state.get("landed") is False and state.get("active") is True and all(state.get(k) == ids.get(k) for k in ids)
        okay = okay and by_key["status"].get("message", {}).get("arming_state") == 2 and by_key["status"].get("message", {}).get("nav_state") == local_mode and by_key["land"].get("message", {}).get("landed") is False
        okay = okay and all(by_key[k] and by_key[k].get("age_wall_s", float("inf")) <= limit for k, limit in (("runtime", .5), ("status", 2), ("land", 5), ("local", .5), ("vio", .5)))
        screenshot_states.append({"side": side, "pass": bool(okay), "runtime": state, "updated_steady_s": snapshot.get("updated_steady_s")})
    png_ok = screenshot.exists() and screenshot.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    check("demo", "actual_airborne_screenshot", png_ok and bool(capture) and
          "true" in str(capture.get("service_response", "")).lower() and all(s["pass"] for s in screenshot_states),
          {"path": str(screenshot), "sha256": digest(screenshot) if png_ok else None, "states": screenshot_states, "service_response": capture.get("service_response")})
    for section in ("phase1", "demo"):
        result[section + "_pass"] = bool(result[section]) and all(c["pass"] is True for c in result[section].values())
        result[section + "_unknown"] = [k for k, c in result[section].items() if c["pass"] is None]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path, help="Completed run directory containing collector/ and flight.ulg")
    parser.add_argument("--output", required=True, type=Path, help="Analysis JSON file")
    args = parser.parse_args()
    try:
        result = analyze(args.run.resolve())
    except Exception as exception:
        result = {"run": str(args.run.resolve()), "phase1_pass": False, "demo_pass": False,
                  "error": f"{type(exception).__name__}: {exception}", "analyzed_utc": utc_now()}
    atomic_json(args.output, result)
    print(json.dumps({"phase1_pass": result["phase1_pass"], "demo_pass": result["demo_pass"],
                      "output": str(args.output), "error": result.get("error")}))
    return 0 if result["phase1_pass"] and result["demo_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
