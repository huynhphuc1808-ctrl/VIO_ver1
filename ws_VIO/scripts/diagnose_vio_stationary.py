#!/usr/bin/env python3
"""Collect bounded disarmed VIO evidence; never create flight-command publishers.

Source scripts/setup.bash first. Start the sensor/PX4 bringup separately, with
no mission backend. Ground truth is deliberately absent from these inputs.
"""

import argparse
import json
import math
from pathlib import Path
import time

from collect_phase1_sitl import atomic_json, plain, versioned_topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=90.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 5 <= args.duration <= 600:
        parser.error("--duration must be finite, between 5 and 600 seconds")

    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition, VehicleStatus
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import String

    args.output.mkdir(parents=True, exist_ok=False)
    stream = (args.output / "samples.jsonl").open("x")
    rclpy.init()
    node = rclpy.create_node("disarmed_vio_diagnostic")
    latest, counts, start = {}, {}, time.monotonic()
    subscriptions = []
    failed = None

    def receive(key, message):
        nonlocal failed
        now = time.monotonic()
        counts[key] = counts.get(key, 0) + 1
        if key in ("left", "right"):
            record = {"header": plain(message.header), "encoding": message.encoding,
                      "width": message.width, "height": message.height}
            if counts[key] == 1 or counts[key] % 100 == 0:
                if message.encoding == "mono8":
                    image = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
                        message.height, message.step)[:, :message.width]
                    features = cv2.goodFeaturesToTrack(image, 200, 0.01, 15)
                    record["corners"] = 0 if features is None else len(features)
                    record["intensity_std"] = float(image.std())
                    cv2.imwrite(str(args.output / (key + ".png")), image)
                stream.write(json.dumps({"key": key, "wall": now, "message": record}) + "\n")
        else:
            record = plain(message, finite=True)
            if key == "vehicle" and message.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                failed = "vehicle armed during disarmed diagnosis"
            stream.write(json.dumps({"key": key, "wall": now, "message": record}) + "\n")
        latest[key] = {"wall": now, "message": record}

    specs = [("left", "/vio/vio_estimator/left/image_raw", Image),
             ("right", "/vio/vio_estimator/right/image_raw", Image),
             ("imu", "/vio/sensors/d435/imu", Imu),
             ("raw", "/vio_msckf/odomimu", Odometry),
             ("visual", "/vio_msckf/poseimu", PoseWithCovarianceStamped),
             ("vio", "/vio/odometry", Odometry), ("health", "/vio/status", String)]
    for key, name, kind in (("local", "vehicle_local_position", VehicleLocalPosition),
                            ("vehicle", "vehicle_status", VehicleStatus),
                            ("fusion", "estimator_status_flags", EstimatorStatusFlags)):
        specs.append((key, versioned_topic("/fmu/out", name, kind), kind))
    for key, topic, kind in specs:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda message, key=key: receive(key, message), qos_profile_sensor_data))
    next_report = start + 1
    snapshots = []
    try:
        while time.monotonic() - start < args.duration and not failed:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now < next_report:
                continue
            next_report = now + 1
            stream.flush()
            commands = {topic: [info.node_name for info in node.get_publishers_info_by_topic(topic)]
                        for topic in ("/fmu/in/vehicle_command", "/fmu/in/vehicle_command_mode_executor")}
            if any(commands.values()):
                failed = "unexpected flight-command publisher during sensor diagnosis"
            snapshot = {"elapsed_wall_s": now - start, "counts": dict(counts),
                        "latest": dict(latest), "command_publishers": commands, "error": failed}
            snapshots.append(snapshot)
            atomic_json(args.output / "latest.json", snapshot)
        atomic_json(args.output / "report.json", {"duration_wall_s": time.monotonic() - start,
                    "error": failed, "counts": counts, "snapshots": snapshots,
                    "note": "Disarmed sensor diagnosis only; this does not accept a flight phase."})
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
