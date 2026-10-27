#!/usr/bin/env python3
"""Bounded negative SITL prearm probe: one normal arm request, never takeoff/select."""

import argparse
from pathlib import Path
import signal
import sys
import time

from collect_phase1_sitl import atomic_json, plain, qos_json, utc_now, versioned_topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="Probe evidence JSON file")
    parser.add_argument("--label", required=True, help="Name of this negative gate scenario")
    parser.add_argument("--gate-id", required=True, type=int,
                        help="Actual registration ID of the global VIO Mission Gate, from collector")
    args = parser.parse_args()
    if not 0 <= args.gate_id <= 255:
        parser.error("--gate-id must be an actual uint8 registration ID")
    if args.output.exists():
        parser.error("--output already exists; preserve earlier probe evidence")

    import json
    import rclpy
    import px4_msgs.msg as px4
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.signals import SignalHandlerOptions

    latest = {}
    registrations = []
    gate_replies = []
    acknowledgements = []
    commands = []
    events = []
    stopped = {"signal": None}
    started = time.monotonic()
    sent_steady = None
    unexpected_arm = False
    previous_handlers = {}
    report = {"schema": "phase1-prearm-probe-v1", "label": args.label,
              "gate_id": args.gate_id, "started_utc": utc_now(),
              "source_component": 191, "commands": commands,
              "registrations": registrations, "gate_replies": gate_replies,
              "acknowledgements": acknowledgements, "events": events,
              "result": "in_progress", "exit_code": None}

    def request_stop(signum, _frame):
        stopped["signal"] = signal.Signals(signum).name

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("phase1_negative_prearm_probe")
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    runtime_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
    command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
    command_topic = versioned_topic("/fmu/in", "vehicle_command", px4.VehicleCommand)
    publisher = node.create_publisher(px4.VehicleCommand, command_topic, command_qos)
    subscriptions = []

    def receive(key, msg):
        now = time.monotonic()
        record = {"receipt_steady_s": now, "receipt_utc": utc_now(), "message": plain(msg)}
        if key == "runtime":
            try:
                record["parsed_json"] = json.loads(msg.data)
            except (ValueError, TypeError):
                record["parsed_json"] = None
        latest[key] = record
        if key == "registration":
            registrations.append(record)
        elif key == "gate" and msg.registration_id == args.gate_id:
            latest["specified_gate"] = record
            gate_replies.append(record)
        elif key == "ack" and msg.command == 400 and msg.target_component == 191:
            if sent_steady is not None and now >= sent_steady:
                acknowledgements.append(record)

    for key, prefix, topic, msg_type in [
        ("runtime", "", "/uav_vio/mission/status", String),
        ("vio_status", "", "/vio/status", String),
        ("vio_odometry", "", "/vio/odometry", Odometry),
        ("status", "/fmu/out", "vehicle_status", px4.VehicleStatus),
        ("land", "/fmu/out", "vehicle_land_detected", px4.VehicleLandDetected),
        ("local", "/fmu/out", "vehicle_local_position", px4.VehicleLocalPosition),
        ("gate", "/fmu/in", "arming_check_reply", px4.ArmingCheckReply),
        ("registration", "/fmu/out", "register_ext_component_reply", px4.RegisterExtComponentReply),
        ("ack", "/fmu/out", "vehicle_command_ack", px4.VehicleCommandAck),
    ]:
        name = versioned_topic(prefix, topic, msg_type) if prefix else topic
        subscriptions.append(node.create_subscription(
            msg_type, name, lambda msg, k=key: receive(k, msg),
            runtime_qos if key == "runtime" else qos))

    def fresh(key, limit=2.0):
        return key in latest and time.monotonic() - latest[key]["receipt_steady_s"] <= limit

    def is_disarmed():
        return fresh("status") and latest["status"]["message"]["arming_state"] == px4.VehicleStatus.ARMING_STATE_DISARMED

    def is_landed():
        return fresh("land", 5.0) and latest["land"]["message"].get("landed") is True

    def command_readers():
        endpoints = node.get_subscriptions_info_by_topic(command_topic)
        snapshot = {
            "receipt_steady_s": time.monotonic(), "receipt_utc": utc_now(),
            "matched_subscription_count": publisher.get_subscription_count(),
            "endpoints": [{"node_name": endpoint.node_name,
                           "node_namespace": endpoint.node_namespace,
                           "topic_type": endpoint.topic_type,
                           "gid": list(endpoint.endpoint_gid),
                           "qos": qos_json(endpoint.qos_profile)} for endpoint in endpoints],
        }
        report["command_readers_pre_send"] = snapshot
        return snapshot

    def prerequisites():
        runtime = latest.get("runtime", {}).get("parsed_json") or {}
        gate = latest.get("specified_gate", {}).get("message", {})
        observed_ids = [entry["message"]["arming_check_id"] for entry in registrations
                        if entry["message"].get("success") and
                        "VIO Mission Gate" in registration_name(entry["message"].get("name", []))]
        readers = command_readers()
        return {
            "fresh_disarmed": is_disarmed(), "fresh_landed": is_landed(),
            "runtime_fresh": fresh("runtime"), "runtime_registered": runtime.get("registered") is True,
            "runtime_not_ready": runtime.get("ready") is False,
            "specified_gate_fresh": fresh("specified_gate"),
            "specified_gate_refuses_arm": gate.get("can_arm_and_run") is False,
            "gate_is_global": all(not gate.get(name, False) for name in (
                "mode_req_local_position", "mode_req_local_position_relaxed",
                "mode_req_global_position", "mode_req_global_position_relaxed")),
            "observed_registration_matches_if_available": not observed_ids or args.gate_id in observed_ids,
            "native_command_reader_discovered": any(
                endpoint["node_name"] == "_CREATED_BY_BARE_DDS_APP_" and
                endpoint["topic_type"] == "px4_msgs/msg/VehicleCommand"
                for endpoint in readers["endpoints"]),
            "command_matched_subscription_count_at_least_two": readers["matched_subscription_count"] >= 2,
        }

    def save():
        report["latest"] = latest
        report["updated_utc"] = utc_now()
        report["elapsed_wall_s"] = time.monotonic() - started
        atomic_json(args.output, report)

    def event(name, **details):
        events.append(dict(event=name, receipt_utc=utc_now(), receipt_steady_s=time.monotonic(), **details))
        save()

    def send(arm):
        command = px4.VehicleCommand()
        command.timestamp = 0  # Match pinned VehicleCommandSender: PX4 timestamps receipt.
        command.command = 400
        command.param1 = 1.0 if arm else 0.0
        command.param2 = 0.0  # Explicitly NONFORCED.
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 191
        command.confirmation = 0
        command.from_external = True
        commands.append({"topic": command_topic, "receipt_steady_s": time.monotonic(),
                         "receipt_utc": utc_now(), "message": plain(command)})
        publisher.publish(command)
        event("arm_sent" if arm else "landed_nonforced_recovery_disarm_sent")

    def spin():
        if stopped["signal"] or not rclpy.ok():
            raise RuntimeError(stopped["signal"] or "ROS shutdown")
        rclpy.spin_once(node, timeout_sec=0.05)

    exit_code = 1
    try:
        save()
        gather_deadline = started + 10.0
        while time.monotonic() < gather_deadline:
            spin()
            if all(prerequisites().values()):
                break
        prereqs = prerequisites()
        report["prerequisites"] = prereqs
        report["precommand_states"] = dict(latest)
        event("prerequisites_checked", conditions=prereqs)
        if not all(prereqs.values()):
            raise RuntimeError("Negative gate prerequisites missing; no arm command sent")
        sent_steady = time.monotonic()
        send(True)  # Exactly one arm request. No select/retry/takeoff path exists.
        ack_deadline = sent_steady + 6.0
        rejection_received = None
        observed_positive_ack = False
        while time.monotonic() < ack_deadline:
            spin()
            if fresh("status") and latest["status"]["message"]["arming_state"] == px4.VehicleStatus.ARMING_STATE_ARMED:
                unexpected_arm = True
                break
            for ack in acknowledgements:
                result = ack["message"]["result"]
                if result == 0:
                    observed_positive_ack = True
                elif result in (1, 2):  # TEMPORARILY_REJECTED or DENIED only.
                    rejection_received = ack
                elif result != 5:  # IN_PROGRESS may precede a terminal ACK.
                    report["unexpected_terminal_ack"] = ack
                    raise RuntimeError(
                        f"Arm ACK result={result} does not demonstrate a native prearm rejection; "
                        "require TEMPORARILY_REJECTED=1 or DENIED=2")
            if observed_positive_ack or rejection_received:
                break
        if unexpected_arm:
            raise RuntimeError("Unexpected armed status during negative prearm probe")
        if observed_positive_ack:
            raise RuntimeError("PX4 accepted arm request; negative gate failed")
        if rejection_received is None:
            raise RuntimeError("No matching rejection ACK within 6 wall seconds")
        report["rejection_ack"] = rejection_received
        event("matching_rejection_ack")
        observe_end = time.monotonic() + 3.0
        while time.monotonic() < observe_end:
            spin()
            if fresh("status") and latest["status"]["message"]["arming_state"] == px4.VehicleStatus.ARMING_STATE_ARMED:
                unexpected_arm = True
                raise RuntimeError("Unexpected armed status after rejection ACK")
            if any(ack["message"]["result"] == 0 for ack in acknowledgements):
                raise RuntimeError("Conflicting accepted ACK after rejection")
            for ack in acknowledgements:
                result = ack["message"]["result"]
                if result not in (1, 2, 5):
                    report["unexpected_terminal_ack"] = ack
                    raise RuntimeError(
                        f"Conflicting arm ACK result={result} after rejection; "
                        "require TEMPORARILY_REJECTED=1 or DENIED=2")
        if not is_disarmed() or not is_landed():
            raise RuntimeError("Final disarmed and landed states are not fresh")
        report["result"] = "PASS_native_arm_rejected_and_remained_disarmed"
        report["observed_disarmed_after_ack_wall_s"] = 3.0
        exit_code = 0
    except Exception as exception:
        report["result"] = "FAIL"
        report["error"] = f"{type(exception).__name__}: {exception}"
        # Even an accepted ACK may precede the armed status. Observe briefly, and
        # issue at most one normal disarm if actual fresh status says armed+landed.
        if sent_steady is not None and rclpy.ok() and not stopped["signal"]:
            recovery_deadline = time.monotonic() + 3.0
            disarm_sent = False
            while time.monotonic() < recovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                armed = (fresh("status") and latest["status"]["message"]["arming_state"] ==
                         px4.VehicleStatus.ARMING_STATE_ARMED)
                if armed:
                    unexpected_arm = True
                    if is_landed() and not disarm_sent:
                        send(False)
                        disarm_sent = True
        print(report["error"], file=sys.stderr, flush=True)
    finally:
        report["unexpected_arm_observed"] = unexpected_arm
        report["final_states"] = dict(latest)
        report["exit_code"] = exit_code
        report["ended_utc"] = utc_now()
        save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    print(f"{args.label}: {report['result']}; evidence={args.output}; exit={exit_code}", flush=True)
    return exit_code


def registration_name(value):
    if isinstance(value, str):
        return value.rstrip("\0")
    return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")


if __name__ == "__main__":
    raise SystemExit(main())
