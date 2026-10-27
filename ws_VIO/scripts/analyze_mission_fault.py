#!/usr/bin/env python3
"""Read-only lifecycle/command and native landing checks for a SITL fault episode."""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

from collect_phase1_sitl import atomic_json, utc_now


def analyze(run):
    from pyulog import ULog

    result = json.loads((run / "result.json").read_text())
    fault = result.get("fault")
    if fault not in ("vio-loss", "stale-local", "takeover", "reselection", "process-death", "progress-timeout", "frame-reset"):
        raise ValueError("This analyzer requires a terminating fault episode")
    when = result["fault_wall_s"]
    checks, rows = {}, defaultdict(list)
    with (run / "collector/messages.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            if row["receipt_steady_s"] < when:
                continue
            topic = row["topic"]
            if topic == "/uav_vio/mission/status":
                row["state"] = row.get("parsed_json") or json.loads(row["message"]["data"])
                rows["runtime"].append(row)
            elif "/in/vehicle_command" in topic:
                rows["commands"].append(row)
            elif "/in/goto_setpoint" in topic:
                rows["goto"].append(row)
            elif "/out/vehicle_local_position" in topic:
                rows["local"].append(row)
            elif topic == "/vio/status":
                rows["vio_status"].append(row)

    def check(name, passed, observed):
        checks[name] = {"pass": bool(passed), "observed": observed}

    check("no_manual_test_recovery", not (run / "manual_recovery.json").exists(), str(run / "manual_recovery.json"))
    check("fault_injected_and_recovered", result.get("fault_injected") and result.get("success"), result.get("error"))
    failed = [r for r in rows["runtime"] if r["state"].get("phase") in ("failed", "interrupted")]
    cutoff = failed[0]["receipt_steady_s"] if failed else when
    check("terminal_failure_or_process_death", bool(failed) or fault == "process-death",
          failed[0]["state"] if failed else None)
    late_goto = [r for r in rows["goto"] if r["receipt_steady_s"] > cutoff + .25]
    late_commands = [r for r in rows["commands"] if r["receipt_steady_s"] > cutoff + .25 and not (
        r["message"]["command"] == 100001 and (r["message"]["param1"] == 18 or
        (fault == "reselection" and r["message"]["source_component"] == 191)))]
    check("cancelled_old_setpoints_and_commands", not late_goto and not late_commands,
          {"late_goto_count": len(late_goto), "late_commands": late_commands})
    check("no_resume_after_terminal", not any(r["state"].get("phase") == "running" and
          r["receipt_steady_s"] > cutoff + .25 for r in rows["runtime"]), cutoff)
    if fault == "progress-timeout":
        check("real_progress_deadline_expired", any("progress deadline exceeded" in r["state"].get("error", "")
              for r in failed), [r["state"].get("error") for r in failed[:1]])
        nearest_local = min(rows["local"], key=lambda r: abs(r["receipt_steady_s"] - cutoff)) if rows["local"] else None
        elapsed = ((nearest_local["message"]["timestamp"] - result["fault_native_timestamp_us"]) / 1e6
                   if nearest_local and result.get("fault_native_timestamp_us") is not None else None)
        check("hold_deadline_elapsed_in_native_sim_time", elapsed is not None and 14.5 <= elapsed <= 17.0, elapsed)
    check("no_rearm", not any(r["message"]["command"] == 400 and r["message"]["param1"] == 1
          for r in rows["commands"]), len(rows["commands"]))
    if fault == "vio-loss":
        check("vio_loss_observed", any(r["message"]["data"].split(":", 1)[0] in ("DEGRADED", "LOST")
              for r in rows["vio_status"]), sorted(set(r["message"]["data"] for r in rows["vio_status"])))
        lands = [r for r in rows["commands"] if r["message"]["command"] == 100001 and r["message"]["param1"] == 18]
        check("one_native_recovery_land", len(lands) == 1, [r["message"] for r in lands])
    if fault == "stale-local" and result.get("fault_action") == "localTakeoff":
        lands = [r for r in rows["commands"] if r["message"]["command"] == 100001 and r["message"]["param1"] == 18]
        check("stale_takeoff_requests_one_native_land", len(lands) == 1, [r["message"] for r in lands])
    if fault == "takeover":
        check("no_backend_command_after_takeover", not [r for r in rows["commands"]
              if r["receipt_steady_s"] > cutoff + .25], len(rows["commands"]))
    if fault == "frame-reset":
        before = json.loads((run / "before_fault.json").read_text())["topics"]["/uav_vio/mission/status"]["parsed_json"]
        relay_log = (run / "local_relay.log").read_text()
        check("frozen_frame_epoch_fault_rejected", before.get("frame_valid") and not before.get("frame_initializing")
              and "mission_only_epoch_reset" in relay_log and bool(failed) and
              failed[0]["state"].get("frame_valid") is False, relay_log)
    if fault == "reselection":
        operator = json.loads((run / "operator_reselection.json").read_text())
        before = operator["before"]["topics"]["/uav_vio/mission/status"]["parsed_json"]
        operator_commands = [r for r in rows["commands"] if r["message"].get("source_component") == 191]
        check("explicit_armed_reselection_not_resume", before.get("armed") and len(operator_commands) == 1 and
              operator_commands[0]["message"].get("param1") == before.get("mode_id") and
              operator_commands[0]["message"].get("command") == 100001 and
              operator.get("acknowledgements", [{}])[-1].get("result") == 0 and
              any("resume" in r["state"].get("error", "") for r in failed), operator.get("acknowledgements"))
    path = run / "flight.ulg"
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    check("ulog_integrity", sha == result.get("ulog_sha256"), sha)
    ulog = ULog(str(path))
    status = ulog.get_dataset("vehicle_status").data
    land = ulog.get_dataset("vehicle_land_detected").data
    check("native_armed_airborne_then_landed_disarmed", any(status["arming_state"] == 2) and
          any(land["landed"] == 0) and status["arming_state"][-1] == 1 and bool(land["landed"][-1]),
          {"final_armed": int(status["arming_state"][-1]), "final_landed": bool(land["landed"][-1]),
           "nav_states": sorted(set(int(s) for s in status["nav_state"]))})
    return {"schema": "json-mission-sitl-fault-analysis-v1", "run": str(run), "fault": fault,
            "fault_action": result.get("fault_action"), "analyzed_utc": utc_now(),
            "pass": all(c["pass"] for c in checks.values()), "checks": checks,
            "limitations": ["One controlled failure case; no claim for untested fault/action combinations.",
                            "250 ms collector inter-topic receipt tolerance for cancellation boundary."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = analyze(args.run.resolve())
    except Exception as error:
        report = {"pass": False, "error": repr(error)}
    atomic_json(args.output, report)
    print(json.dumps({"pass": report["pass"], "failed_checks": [k for k, v in report.get("checks", {}).items()
                    if not v["pass"]], "error": report.get("error")}))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
