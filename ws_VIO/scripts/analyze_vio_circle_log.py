#!/usr/bin/env python3
"""Read-only acceptance analysis for a PX4 VIO-only circle ULog.

The default IDs (executor 1, local mode 23) reproduce the legacy circle baseline.
Pass the IDs reported by /uav_vio/mission/status for the JSON mission runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

RADIAL_BAND_M = 0.5
LEGACY_EXECUTOR_ID = 1
LEGACY_LOCAL_MODE_ID = 23
MODE_EXECUTOR_COMPONENT_START = 1000
# Pinned PX4 ModeManagement.hpp: five executors and eight external modes.
EXECUTOR_ID_RANGE = (1, 5)
LOCAL_MODE_ID_RANGE = (23, 30)
NAV_TAKEOFF = 17
NAV_ORBIT = 21
NAV_LAND = 18
ARMING_DISARMED = 1


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def dataset(ulog: Any, name: str) -> dict[str, Any]:
    return ulog.get_dataset(name).data


def nearest_index(values: Any, timestamp: int) -> int:
    # On an exact half-sample tie choose the sample at/after the command time.
    return min(range(len(values)), key=lambda i: (abs(int(values[i]) - timestamp), int(values[i]) < timestamp))


def positive_finite(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive finite number") from exc
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def bounded_integer(minimum: int, maximum: int):
    def parse(value: str) -> int:
        try:
            number = int(value)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"must be an integer in {minimum}..{maximum}") from exc
        if not minimum <= number <= maximum:
            raise argparse.ArgumentTypeError(f"must be an integer in {minimum}..{maximum}")
        return number
    return parse


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise RuntimeError("pyulog is required (python3 -m pip install pyulog)") from exc

    path = Path(args.input).expanduser().resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    ulog = ULog(str(path))
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, observed: Any, expected: Any) -> None:
        checks[name] = {"pass": bool(passed), "observed": observed, "expected": expected}

    commands = dataset(ulog, "vehicle_command")
    local = dataset(ulog, "vehicle_local_position")
    vehicle_status = dataset(ulog, "vehicle_status")
    land_detected = dataset(ulog, "vehicle_land_detected")
    estimator = dataset(ulog, "estimator_status_flags")
    params = ulog.initial_parameters
    mission_source_component = MODE_EXECUTOR_COMPONENT_START + args.executor_id

    # MAV_CMD_DO_ORBIT=34 captures its center from the nearest local-position sample.
    orbit_indices = [i for i, c in enumerate(commands["command"]) if int(c) == 34]
    if not orbit_indices:
        raise RuntimeError("ULog has no vehicle_command DO_ORBIT (34)")
    if args.orbit_index >= len(orbit_indices):
        raise RuntimeError(f"requested Orbit index {args.orbit_index}, but ULog contains {len(orbit_indices)}")
    orbit_i = orbit_indices[args.orbit_index]
    orbit_t = int(commands["timestamp"][orbit_i])
    orbit_param1 = float(commands["param1"][orbit_i])
    orbit_param2 = float(commands["param2"][orbit_i])
    center_i = nearest_index(local["timestamp"], orbit_t)
    center_x, center_y = float(local["x"][center_i]), float(local["y"][center_i])

    end_indices = [
        i for i in range(orbit_i + 1, len(commands["command"]))
        if int(commands["command"][i]) == 100001 and math.isclose(float(commands["param1"][i]), args.local_mode_id)
    ]
    if not end_indices:
        raise RuntimeError(f"ULog has no post-orbit command 100001 param1={args.local_mode_id} schedule marker")
    end_t = int(commands["timestamp"][end_indices[0]])
    window = [i for i, t in enumerate(local["timestamp"]) if orbit_t <= int(t) <= end_t]
    if not window:
        raise RuntimeError("no vehicle_local_position samples in Orbit interval")

    takeoff_indices = [i for i, c in enumerate(commands["command"]) if int(c) == 22]
    armed_indices = [i for i, state in enumerate(vehicle_status["arming_state"]) if int(state) == 2]
    if not takeoff_indices or not armed_indices:
        raise RuntimeError("ULog must contain native Takeoff command and armed vehicle_status")
    takeoff_t = int(commands["timestamp"][takeoff_indices[0]])
    arm_t = int(vehicle_status["timestamp"][armed_indices[0]])
    takeoff_local_i = nearest_index(local["timestamp"], takeoff_t)
    arm_local_i = nearest_index(local["timestamp"], arm_t)
    start_z = float(local["z"][takeoff_local_i])
    arm_z = float(local["z"][arm_local_i])

    positions: list[tuple[int, float, float, float, float]] = []
    for i in window:
        x, y, z = float(local["x"][i]), float(local["y"][i]), float(local["z"][i])
        r = math.hypot(x - center_x, y - center_y)
        positions.append((int(local["timestamp"][i]), x, y, z, r))

    acquired_at = next((i for i, p in enumerate(positions) if abs(p[4] - args.radius) <= args.radial_band), None)
    if acquired_at is None:
        raise RuntimeError("Orbit never acquired the configured radial band")
    measured = positions[acquired_at:]
    radii = [p[4] for p in measured]
    zs = [p[3] for p in measured]

    # atan2(East, North) increases clockwise in NED. Unwrap each sample delta.
    angles = [math.atan2(p[2] - center_y, p[1] - center_x) for p in measured]
    angle_sum = 0.0
    for previous, current in zip(angles, angles[1:]):
        delta = current - previous
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        angle_sum += delta
    direction_sign = 1.0 if args.direction == "clockwise" else -1.0
    directed_angle = direction_sign * angle_sum
    rmse = math.sqrt(statistics.fmean((r - args.radius) ** 2 for r in radii))
    signed_radius_expected = direction_sign * args.radius
    orbit_radius_error = abs(orbit_param1 - signed_radius_expected)
    orbit_speed_error = abs(orbit_param2 - args.speed)
    orbit_params_ok = (
        finite(orbit_param1) and finite(orbit_param2)
        and orbit_radius_error <= 0.01 and orbit_speed_error <= 0.01
    )
    check("orbit_command_parameters", orbit_params_ok,
          {"command_param1_signed_radius_m": orbit_param1,
           "expected_signed_radius_m": signed_radius_expected,
           "radius_error_m": orbit_radius_error,
           "command_param2_speed_m_s": orbit_param2,
           "expected_speed_m_s": args.speed,
           "speed_error_m_s": orbit_speed_error,
           "tolerance_m": 0.01, "tolerance_m_s": 0.01},
          "native Orbit param1 signed radius and param2 speed match configuration within 0.01")

    if args.expected_sha256 is not None:
        check("ulog_sha256", digest.lower() == args.expected_sha256.lower(), digest, args.expected_sha256.lower())
    gps_keys = ["SYS_HAS_GPS", "SIM_GZ_EN_GPS", "SIM_GPS_USED", "SENS_EN_GPSSIM", "EKF2_GPS_CTRL"]
    gps_values = {key: params.get(key) for key in gps_keys}
    gps_disabled = (
        params.get("SYS_HAS_GPS") == 0
        and params.get("SIM_GZ_EN_GPS") == 0
        and params.get("EKF2_GPS_CTRL") == 0
        and params.get("SIM_GPS_USED", 0) == 0
        and params.get("SENS_EN_GPSSIM", 0) == 0
    )
    check("gps_disabled", gps_disabled, gps_values, "all present GPS enable/fusion values are 0")
    check("ev_ctrl", params.get("EKF2_EV_CTRL") == 7, params.get("EKF2_EV_CTRL"), 7)

    native_failsafe_keys = [
        "COM_FAIL_ACT_T", "COM_POS_FS_EPH", "COM_VEL_FS_EVH", "COM_LOW_BAT_ACT",
        "COM_POS_LOW_ACT", "NAV_DLL_ACT", "NAV_RCL_ACT", "GF_ACTION",
    ]
    failsafe_params = {key: params.get(key) for key in native_failsafe_keys}
    check("native_failsafe_params_finite", all(finite(params.get(k)) for k in native_failsafe_keys),
          failsafe_params, "all listed native failsafe parameters finite")

    # External estimator fusion must remain active and GNSS fusion absent over Orbit.
    estimator_window = [i for i, t in enumerate(estimator["timestamp"]) if orbit_t <= int(t) <= end_t]
    ev_fields = ["cs_ev_pos", "cs_ev_vel", "cs_ev_hgt"]
    gnss_fields = ["cs_gnss_pos", "cs_gnss_vel"]
    ev_ok = bool(estimator_window) and all(all(bool(estimator[k][i]) for k in ev_fields) for i in estimator_window)
    gnss_ok = bool(estimator_window) and all(not bool(estimator[k][i]) for k in gnss_fields for i in estimator_window)
    estimator_observed = {
        "samples": len(estimator_window),
        "ev_pos_vel_hgt_all_true": ev_ok,
        "ev_yaw_all_false": bool(estimator_window) and all(not bool(estimator["cs_ev_yaw"][i]) for i in estimator_window),
        "gnss_pos_vel_all_false": gnss_ok,
    }
    check("estimator_fusion", ev_ok and gnss_ok and estimator_observed["ev_yaw_all_false"],
          estimator_observed, "EV pos/vel/hgt true; EV yaw and GNSS pos/vel false during Orbit")

    observed_commands = []
    # The normal-channel bootstrap selects the executor's owned mission mode
    # before ownership exists. Only the local-mode and Land schedule targets
    # belong to this executor-command check, so bootstrap selection is excluded.
    required_command_specs = {
        "arm": lambda i: int(commands["command"][i]) == 400,
        "takeoff": lambda i: int(commands["command"][i]) == 22,
        "orbit": lambda i: int(commands["command"][i]) == 34,
        "schedule": lambda i: int(commands["command"][i]) == 100001 and math.isclose(float(commands["param1"][i]), args.local_mode_id),
        "land": lambda i: int(commands["command"][i]) == 100001 and math.isclose(float(commands["param1"][i]), 18.0),
    }
    command_sources: dict[str, list[int]] = {}
    for label, predicate in required_command_specs.items():
        ids = [i for i in range(len(commands["command"])) if predicate(i)]
        command_sources[label] = [int(commands["source_component"][i]) for i in ids]
        observed_commands.extend({"kind": label, "command": int(commands["command"][i]),
                                 "param1": (float(commands["param1"][i]) if finite(commands["param1"][i]) else None),
                                 "param2": (float(commands["param2"][i]) if finite(commands["param2"][i]) else None),
                                 "source_component": int(commands["source_component"][i])} for i in ids)
    source_ok = all(sources and all(s == mission_source_component for s in sources)
                    for sources in command_sources.values())
    check("mission_command_source", source_ok, command_sources,
          f"arm/takeoff/orbit/schedule/land all from source_component {mission_source_component}")

    arming_values = [int(x) for x in vehicle_status["arming_state"]]
    nav_values = [int(x) for x in vehicle_status["nav_state"]]
    final_disarmed = bool(arming_values) and arming_values[-1] == ARMING_DISARMED
    final_landed_state = len(land_detected["landed"]) > 0 and bool(land_detected["landed"][-1])
    landed_t = int(land_detected["timestamp"][-1]) if len(land_detected["timestamp"]) else None
    disarmed_t = int(vehicle_status["timestamp"][-1]) if len(vehicle_status["timestamp"]) else None
    landed_disarmed_delta_us = abs(landed_t - disarmed_t) if landed_t is not None and disarmed_t is not None else None
    final_landed = final_landed_state and final_disarmed and landed_disarmed_delta_us <= 1_000_000
    nav_ok = all(value in nav_values for value in (NAV_TAKEOFF, NAV_ORBIT, NAV_LAND))
    check("flight_states", nav_ok and final_landed,
          {"nav_states_seen": sorted(set(nav_values)), "arming_states_seen": sorted(set(arming_values)),
           "final_arming_state": arming_values[-1] if arming_values else None,
           "final_disarmed": final_disarmed, "land_state_seen": NAV_LAND in nav_values,
           "final_landed_detected": bool(land_detected["landed"][-1]) if len(land_detected["landed"]) else None,
           "final_land_timestamp_us": landed_t, "final_disarmed_timestamp_us": disarmed_t,
           "landed_disarmed_delta_us": landed_disarmed_delta_us},
          {"nav_states_include": [NAV_TAKEOFF, NAV_ORBIT, NAV_LAND], "final_arming_state": ARMING_DISARMED,
           "final_landed_detected": True, "landed_disarmed_delta_max_us": 1_000_000})
    failsafe_seen = any(bool(x) for x in vehicle_status["failsafe"])
    check("no_failsafe", not failsafe_seen, failsafe_seen, False)

    check("radial_band_acquired", acquired_at is not None,
          {"first_sample_index": acquired_at, "samples_through_orbit_end": len(measured),
           "tolerance_m": args.radial_band}, "at least one sample within radius +/- radial band")
    check("lap_angle", directed_angle >= 2.0 * math.pi * args.laps,
          {"signed_angle_rad": directed_angle, "laps": directed_angle / (2.0 * math.pi)},
          f"at least {args.laps} directed laps")
    check("radius_containment", max(abs(r - args.radius) for r in radii) <= args.radial_band,
          {"mean_m": statistics.fmean(radii), "min_m": min(radii), "max_m": max(radii),
           "rmse_m": rmse, "radial_band_m": args.radial_band},
          f"all acquired samples within {args.radial_band} m of target radius")
    z_pp = max(zs) - min(zs)
    orbit_altitude_above_start = start_z - statistics.fmean(zs)
    start_reference_ok = (
        finite(start_z) and finite(arm_z)
        and abs(takeoff_t - arm_t) <= 1_000_000
        and abs(start_z - arm_z) <= 0.5
    )
    check("start_z_reference", start_reference_ok,
          {"takeoff_command_timestamp_us": takeoff_t, "arming_timestamp_us": arm_t,
           "takeoff_local_z_ned_m": start_z, "arming_local_z_ned_m": arm_z,
           "takeoff_arming_time_delta_us": abs(takeoff_t - arm_t),
           "takeoff_arming_z_delta_m": abs(start_z - arm_z)},
          "finite local z near Takeoff and arming (time delta <= 1 s, z delta <= 0.5 m)")
    altitude_error = abs(orbit_altitude_above_start - args.altitude)
    check("altitude_above_start", start_reference_ok and altitude_error <= 0.5,
          {"configured_altitude_m": args.altitude, "start_z_ned_m": start_z,
           "orbit_mean_z_ned_m": statistics.fmean(zs),
           "orbit_altitude_above_start_m": orbit_altitude_above_start,
           "absolute_error_m": altitude_error, "tolerance_m": 0.5},
          "orbit mean altitude above takeoff/start local z within 0.5 m")
    metrics = {
        "center_north_m": center_x,
        "center_east_m": center_y,
        "orbit_start_timestamp_us": orbit_t,
        "orbit_end_timestamp_us": end_t,
        "radial_band_acquired_timestamp_us": measured[0][0],
        "lap_angle_rad": directed_angle,
        "laps": directed_angle / (2.0 * math.pi),
        "radius_mean_m": statistics.fmean(radii),
        "radius_min_m": min(radii),
        "radius_max_m": max(radii),
        "radius_rmse_m": rmse,
        "orbit_z_mean_ned_m": statistics.fmean(zs),
        "orbit_z_peak_to_peak_m": z_pp,
        "orbit_position_samples": len(measured),
        "start_z_ned_m": start_z,
        "orbit_altitude_above_start_m": orbit_altitude_above_start,
    }
    check("altitude_stability", z_pp <= 0.5, {"z_mean_ned_m": metrics["orbit_z_mean_ned_m"], "peak_to_peak_m": z_pp},
          "NED z peak-to-peak <= 0.5 m")
    truth_metrics = None
    if args.groundtruth:
        # Standard simulator log output, read after flight only. These samples
        # never become a VIO/flight-estimator/controller measurement.
        truth = dataset(ulog, "vehicle_local_position_groundtruth")
        gt_center = nearest_index(truth["timestamp"], orbit_t)
        gt_ground = nearest_index(truth["timestamp"], takeoff_t)
        gt_rows = []
        for i, timestamp in enumerate(truth["timestamp"]):
            if orbit_t <= int(timestamp) <= end_t:
                x, y, z = (float(truth[k][i]) for k in ("x", "y", "z"))
                radius = math.hypot(x - float(truth["x"][gt_center]), y - float(truth["y"][gt_center]))
                gt_rows.append((i, x, y, z, radius))
        acquired = next((i for i, row in enumerate(gt_rows) if abs(row[4] - args.radius) <= args.radial_band), None)
        if acquired is None:
            raise RuntimeError("physical simulator model never acquired the Orbit radial band")
        gt_rows = gt_rows[acquired:]
        gt_angles = [math.atan2(row[2] - float(truth["y"][gt_center]), row[1] - float(truth["x"][gt_center])) for row in gt_rows]
        angle = sum(math.atan2(math.sin(b - a), math.cos(b - a)) for a, b in zip(gt_angles, gt_angles[1:]))
        gt_radii = [r[4] for r in gt_rows]
        gt_z = [r[3] for r in gt_rows]
        speeds = [math.hypot(float(truth["vx"][r[0]]), float(truth["vy"][r[0]])) for r in gt_rows]
        truth_metrics = {"laps": direction_sign * angle / (2 * math.pi),
                         "radius_rmse_m": math.sqrt(statistics.fmean((r - args.radius) ** 2 for r in gt_radii)),
                         "radius_min_m": min(gt_radii), "radius_max_m": max(gt_radii),
                         "speed_median_m_s": statistics.median(speeds),
                         "z_peak_to_peak_m": max(gt_z) - min(gt_z),
                         "altitude_m": float(truth["z"][gt_ground]) - statistics.fmean(gt_z)}
        check("physical_orbit_radius", min(gt_radii) >= args.radius - args.radial_band and
              max(gt_radii) <= args.radius + args.radial_band and truth_metrics["radius_rmse_m"] <= args.radial_band,
              truth_metrics, "physical radius within the same configured radial band")
        check("physical_orbit_direction_laps", .95 * args.laps <= truth_metrics["laps"] <= args.laps + .25,
              truth_metrics["laps"], "physical direction/laps match request")
        check("physical_orbit_speed", abs(truth_metrics["speed_median_m_s"] - args.speed) <= .3,
              truth_metrics["speed_median_m_s"], "physical median speed within 0.3 m/s")
        check("physical_orbit_altitude", truth_metrics["z_peak_to_peak_m"] <= .5 and
              abs(truth_metrics["altitude_m"] - args.altitude) <= .5,
              truth_metrics, "physical altitude error/peak-to-peak <= 0.5 m")
    failed = [name for name, item in checks.items() if not item["pass"]]
    return {
        "input": str(path),
        "input_sha256": digest,
        "configuration": {"radius_m": args.radius, "altitude_m": args.altitude, "laps": args.laps,
                          "speed_m_s": args.speed, "direction": args.direction, "radial_band_m": args.radial_band,
                          "expected_sha256": args.expected_sha256,
                          "executor_id": args.executor_id, "local_mode_id": args.local_mode_id,
                          "orbit_index": args.orbit_index,
                          "mission_source_component": mission_source_component},
        "metrics": metrics,
        "evaluation_only_groundtruth_metrics": truth_metrics,
        "initial_parameters": {key: params.get(key) for key in gps_keys + ["EKF2_EV_CTRL"] + native_failsafe_keys},
        "mission_commands": observed_commands,
        "checks": checks,
        "valid": not failed,
        "failed_checks": failed,
    }


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="PX4 ULog path")
    parser.add_argument("--radius", type=positive_finite, default=2.0)
    parser.add_argument("--altitude", type=positive_finite, default=2.0, help="configured local altitude in meters")
    parser.add_argument("--speed", type=positive_finite, default=1.0, help="configured native Orbit speed in m/s")
    parser.add_argument("--laps", type=positive_finite, default=1.0)
    parser.add_argument("--direction", choices=("clockwise", "counterclockwise"), default="clockwise")
    parser.add_argument("--radial-band", type=positive_finite, default=RADIAL_BAND_M)
    parser.add_argument("--executor-id", type=bounded_integer(*EXECUTOR_ID_RANGE), default=LEGACY_EXECUTOR_ID,
                        help="registered executor ID (1..5; legacy baseline default: 1)")
    parser.add_argument("--local-mode-id", type=bounded_integer(*LOCAL_MODE_ID_RANGE), default=LEGACY_LOCAL_MODE_ID,
                        help="registered local Goto nav_state (23..30; legacy baseline default: 23)")
    parser.add_argument("--orbit-index", type=bounded_integer(0, 1023), default=0,
                        help="zero-based native Orbit command to analyze (default: first)")
    parser.add_argument("--groundtruth", action="store_true", help="also verify physical Orbit from evaluation-only simulator ULog")
    parser.add_argument("--expected-sha256", help="optional expected input SHA256; omit to skip hash comparison")
    parser.add_argument("--output", help="write JSON report to this path")
    args = parser.parse_args(argv)
    if args.expected_sha256 is not None and (len(args.expected_sha256) != 64 or any(c not in "0123456789abcdefABCDEF" for c in args.expected_sha256)):
        parser.error("--expected-sha256 must be 64 hexadecimal characters")
    return args


def main() -> int:
    args = parse_arguments()
    try:
        report = analyze(args)
    except Exception as exc:  # Report malformed/missing ULog data as a failed analyzer run.
        print(f"analyzer error: {exc}", file=sys.stderr)
        return 2
    rendered = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
