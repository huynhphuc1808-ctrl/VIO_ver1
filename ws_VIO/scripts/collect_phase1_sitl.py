#!/usr/bin/env python3
"""Bounded, read-only Phase 1 SITL evidence collector (source setup.bash first)."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from datetime import datetime, timezone


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def plain(value, finite=False):
    """Convert generated ROS messages/arrays to JSON, retaining NaN in raw logs."""
    if isinstance(value, float):
        return None if finite and not math.isfinite(value) else value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, dict):
        return {str(key): plain(item, finite) for key, item in value.items()}
    if hasattr(value, "get_fields_and_field_types"):
        return {
            key: plain(getattr(value, key), finite)
            for key in value.get_fields_and_field_types()
        }
    if hasattr(value, "tolist"):
        return plain(value.tolist(), finite)
    if isinstance(value, (list, tuple, bytes, bytearray)) or hasattr(value, "__iter__"):
        return [plain(item, finite) for item in value]
    return str(value)


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(plain(value, finite=True), stream, indent=2, allow_nan=False)
        stream.write("\n")
    os.replace(temporary, path)


def versioned_topic(prefix, name, msg_type):
    version = int(getattr(msg_type, "MESSAGE_VERSION", 0))
    return f"{prefix}/{name}" + (f"_v{version}" if version else "")


def topic_specs():
    # Resolve suffixes from the installed pinned types, never a guessed ROS name.
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    import px4_msgs.msg as px4

    specs = [
        ("/uav_vio/mission/status", String),
        ("/vio/status", String),
        ("/vio/odometry", Odometry),
        ("/clock", Clock),
    ]
    for direction, name, typename in [
        ("in", "vehicle_visual_odometry", "VehicleOdometry"),
        ("out", "vehicle_local_position", "VehicleLocalPosition"),
        ("out", "vehicle_status", "VehicleStatus"),
        ("out", "vehicle_land_detected", "VehicleLandDetected"),
        ("out", "estimator_status_flags", "EstimatorStatusFlags"),
        ("out", "failsafe_flags", "FailsafeFlags"),
        ("out", "vehicle_global_position", "VehicleGlobalPosition"),
        ("in", "vehicle_command", "VehicleCommand"),
        ("in", "vehicle_command_mode_executor", "VehicleCommand"),
        ("in", "goto_setpoint", "GotoSetpoint"),
        ("in", "config_control_setpoints", "VehicleControlMode"),
        ("in", "arming_check_reply", "ArmingCheckReply"),
        ("out", "register_ext_component_reply", "RegisterExtComponentReply"),
        ("in", "register_ext_component_request", "RegisterExtComponentRequest"),
        ("out", "arming_check_request", "ArmingCheckRequest"),
        ("out", "vehicle_command_ack", "VehicleCommandAck"),
    ]:
        msg_type = getattr(px4, typename)
        specs.append((versioned_topic(f"/fmu/{direction}", name, msg_type), msg_type))
    return specs


def type_name(msg_type):
    package = msg_type.__module__.split(".")[0]
    return f"{package}/msg/{msg_type.__name__}"


def qos_json(qos):
    result = {}
    for field in ("history", "depth", "reliability", "durability", "liveliness",
                  "deadline", "lifespan", "liveliness_lease_duration"):
        value = getattr(qos, field, None)
        result[field] = (value.nanoseconds if hasattr(value, "nanoseconds") else
                         int(value) if isinstance(value, int) else str(value))
    return result


def graph_snapshot(node, monitored):
    result = {
        "receipt_utc": utc_now(), "receipt_steady_s": time.monotonic(),
        "nodes": [{"name": name, "namespace": namespace}
                  for name, namespace in node.get_node_names_and_namespaces()],
        "topics": dict(node.get_topic_names_and_types()), "monitored": {},
    }
    for topic in monitored:
        endpoints = {}
        for kind, query in (("publishers", node.get_publishers_info_by_topic),
                            ("subscriptions", node.get_subscriptions_info_by_topic)):
            try:
                endpoints[kind] = [{
                    "node_name": entry.node_name, "node_namespace": entry.node_namespace,
                    "topic_type": entry.topic_type, "qos": qos_json(entry.qos_profile),
                    "gid": list(entry.endpoint_gid),
                } for entry in query(topic)]
            except Exception as exception:
                endpoints[kind] = {"error": str(exception)}
        result["monitored"][topic] = endpoints
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="Evidence directory")
    parser.add_argument("--duration", type=float, default=1200.0,
                        help="Maximum wall seconds, independent of simulated time (default: 1200)")
    parser.add_argument("--snapshot-interval", type=float, default=1.0,
                        help="Wall seconds between latest.json snapshots (default: 1)")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if not math.isfinite(args.snapshot_interval) or not 0.05 <= args.snapshot_interval <= 5:
        parser.error("--snapshot-interval must be finite, between 0.05 and 5 seconds")

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.signals import SignalHandlerOptions

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # A new run must never silently truncate earlier evidence.
    stream = (output / "messages.jsonl").open("x", encoding="utf-8")
    start = time.monotonic()
    started_utc = utc_now()
    stop = {"reason": None}
    previous_handlers = {}

    def request_stop(signum, _frame):
        stop["reason"] = signal.Signals(signum).name

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("phase1_sitl_evidence_collector")
    subscriptions = []
    specs = topic_specs()
    latest = {}
    stats = {topic: {"type": type_name(msg_type), "count": 0, "first_steady_s": None,
                     "last_steady_s": None, "max_receipt_gap_s": 0.0}
             for topic, msg_type in specs}

    def receive(topic, msg_type, msg):
        now = time.monotonic()
        record = {"topic": topic, "type": type_name(msg_type),
                  "receipt_steady_s": now, "since_start_s": now - start,
                  "receipt_utc": utc_now(), "message": plain(msg)}
        if msg_type.__name__ == "String":
            try:
                record["parsed_json"] = json.loads(msg.data)
            except (ValueError, TypeError):
                pass
        stream.write(json.dumps(record, allow_nan=True, separators=(",", ":")) + "\n")
        current = stats[topic]
        if current["last_steady_s"] is not None:
            current["max_receipt_gap_s"] = max(current["max_receipt_gap_s"],
                                                now - current["last_steady_s"])
        if not current["count"]:
            current["first_steady_s"] = now
        current["last_steady_s"] = now
        current["count"] += 1
        latest[topic] = record

    for topic, msg_type in specs:
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=(DurabilityPolicy.TRANSIENT_LOCAL
                                     if topic == "/uav_vio/mission/status"
                                     else DurabilityPolicy.VOLATILE))
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, t=topic, m=msg_type: receive(t, m, msg), qos))

    def report(active, reason=None):
        now = time.monotonic()
        elapsed = now - start
        rates = {}
        for topic, current in stats.items():
            value = dict(current)
            interval = ((value["last_steady_s"] - value["first_steady_s"])
                        if value["count"] > 1 else 0.0)
            value.update({
                "rate_over_run_hz": value["count"] / elapsed if elapsed else 0.0,
                "rate_while_receiving_hz": (value["count"] - 1) / interval if interval else 0.0,
                "last_age_wall_s": now - value["last_steady_s"] if value["count"] else None,
            })
            rates[topic] = value
        return {"schema": "phase1-sitl-evidence-v1", "active": active, "stop_reason": reason,
                "started_utc": started_utc, "updated_utc": utc_now(),
                "start_steady_s": start, "updated_steady_s": now, "elapsed_wall_s": elapsed,
                "duration_limit_wall_s": args.duration,
                "environment": {key: os.environ.get(key) for key in (
                    "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
                    "GZ_PARTITION", "PX4_DIR", "PX4_WS", "VIO_ALGORITHM_WS")},
                "topics": {topic: dict(value, age_wall_s=now-value["receipt_steady_s"])
                           for topic, value in latest.items()}, "statistics": rates}

    exit_code = 0
    last_refresh = 0.0
    last_graph = 0.0
    try:
        atomic_json(output / "latest.json", report(True))
        print(f"Collecting {len(specs)} topics into {output}; maximum {args.duration:g} wall s", flush=True)
        while not stop["reason"] and rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_refresh >= args.snapshot_interval:
                stream.flush()
                atomic_json(output / "latest.json", report(True))
                last_refresh = now
            if now - last_graph >= 15.0:
                atomic_json(output / "graph_latest.json", graph_snapshot(node, stats))
                last_graph = time.monotonic()
        stop["reason"] = stop["reason"] or ("duration_limit" if rclpy.ok() else "ROS_shutdown")
    except Exception as exception:
        stop["reason"] = f"exception: {type(exception).__name__}: {exception}"
        exit_code = 1
        print(stop["reason"], file=sys.stderr, flush=True)
    finally:
        try:
            atomic_json(output / "graph_final.json", graph_snapshot(node, stats))
        except Exception as exception:
            atomic_json(output / "graph_final.json", {"error": str(exception)})
            exit_code = 1
        final = report(False, stop["reason"])
        final["exit_code"] = exit_code
        atomic_json(output / "latest.json", final)
        atomic_json(output / "summary.json", final)
        stream.flush()
        os.fsync(stream.fileno())
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    print(f"Collector stopped: {stop['reason']}; exit={exit_code}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
