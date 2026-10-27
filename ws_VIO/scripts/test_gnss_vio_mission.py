#!/usr/bin/env python3
"""SITL-only test fixture: upload PX4 Mission and inject native GNSS/VIO loss.

This is a test driver, not a mission navigator or flight-time ROS service.
Run only against an isolated, already running ws_VIO SITL launch.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', type=Path, required=True)
    parser.add_argument('--launch-log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-sitl-arm', action='store_true')
    parser.add_argument('--waypoint-side-m', type=float, default=2.0,
                        help='Side length within the feature-wall clearance, in meters')
    parser.add_argument('--loops', type=int, default=12)
    args = parser.parse_args()
    if not math.isfinite(args.waypoint_side_m) or not 1 <= args.waypoint_side_m <= 2:
        parser.error('waypoint side must be finite and between 1 and 2 m for this scene')
    if not 3 <= args.loops <= 20:
        parser.error('loops must be between 3 and 20')
    if not args.allow_sitl_arm:
        parser.error('This flight test requires --allow-sitl-arm')
    binaries = args.px4_dir.resolve() / 'build/px4_sitl_default/bin'
    connection = None
    results = {}
    last_heartbeat = 0.0

    def commandline(name, *arguments):
        result = subprocess.run([str(binaries / ('px4-' + name)), '--instance', '0', *arguments],
                                capture_output=True, text=True, timeout=4)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        return result.stdout

    def snapshot(topic):
        text = commandline('listener', topic, '-n', '1')
        values = {}
        for key, value in re.findall(r'^\s*(\w+):\s*(.+)$', text, re.M):
            value = value.split(' ')[0]
            if value.lower() in ('true', 'false'):
                values[key] = value.lower() == 'true'
            else:
                try:
                    values[key] = float(value)
                except ValueError:
                    values[key] = value
        return values

    def heartbeat():
        nonlocal last_heartbeat
        if connection and time.monotonic() - last_heartbeat > 0.8:
            connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_heartbeat = time.monotonic()

    def save(name, value):
        results[name] = value
        args.output.write_text(json.dumps(results, indent=2) + '\n')
        print(name, json.dumps(value), flush=True)

    def wait_for(name, predicate, timeout=65, sustained=0):
        deadline = time.monotonic() + timeout
        since = None
        while time.monotonic() < deadline:
            heartbeat()
            try:
                value = predicate()
            except (RuntimeError, subprocess.TimeoutExpired):
                value = False
            if value:
                since = since or time.monotonic()
                if time.monotonic() - since >= sustained:
                    save(name, value)
                    return value
            else:
                since = None
            time.sleep(0.15)
        raise RuntimeError('Timeout waiting for ' + name)

    def joint_fusion():
        f, p = snapshot('estimator_status_flags'), snapshot('vehicle_local_position')
        return all(f.get(k) for k in ('cs_gnss_pos', 'cs_gnss_vel', 'cs_ev_vel', 'cs_ev_hgt')) and all(
            p.get(k) for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'xy_global'))

    def send_command(command, *params):
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                        command, 0, *(list(params) + [0] * (7 - len(params))))
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            heartbeat()
            ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.3)
            if ack and ack.command == command:
                if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise RuntimeError('PX4 rejected command: ' + str(ack))
                return
        raise RuntimeError('PX4 command ACK timeout')

    try:
        wait_for('startup', lambda: 'EKF2_GPS_MODE' in commandline('param', 'show', 'EKF2_GPS_MODE'), 30)
        wait_for('joint_fusion_before_arm', joint_fusion, 100, sustained=5)
        # Test uses only the local SITL endpoint, after checking its native console.
        connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=245)
        if not connection.wait_heartbeat(timeout=10):
            raise RuntimeError('No local SITL MAVLink heartbeat')
        g = snapshot('vehicle_global_position')
        commandline('param', 'set', 'COM_RC_IN_MODE', '4')  # no RC transmitter in this test
        commandline('param', 'set', 'MPC_XY_CRUISE', '1.0')
        side = args.waypoint_side_m
        points = [(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)]
        points += [(n, e, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
                   for n, e in [(side, 0), (side, side), (0, side), (0, 0)] * args.loops]
        points += [(0, 0, mavutil.mavlink.MAV_CMD_NAV_LAND)]
        save('mission_fixture', {'side_m': side, 'loops': args.loops,
             'altitude_m': 2.0, 'items': points,
             'reason': 'Keep the native Navigator regression inside the feature-wall clearance.'})
        items = []
        for index, (north, east, cmd) in enumerate(points):
            lat = int((g['lat'] + north / 111111.0) * 1e7)
            lon = int((g['lon'] + east / (111111.0 * math.cos(math.radians(g['lat'])))) * 1e7)
            items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                connection.target_system, connection.target_component, index,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd,
                int(index == 0), 1, 0, 0.5, 0, math.nan, lat, lon, 2.0))
        connection.mav.mission_count_send(connection.target_system, connection.target_component, len(items))
        deadline = time.monotonic() + 20
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError('Mission upload timeout')
            heartbeat()
            request = connection.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                                            blocking=True, timeout=0.3)
            if not request:
                continue
            if request.get_type() == 'MISSION_ACK':
                if request.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError('Mission rejected: ' + str(request))
                break
            connection.mav.send(items[request.seq])
        wait_for('px4_preflight_checks', lambda: snapshot('vehicle_status').get('pre_flight_checks_pass')
                 and not snapshot('vehicle_status').get('gcs_connection_lost'), 25, sustained=2)
        send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        send_command(mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
        wait_for('mission_airborne', lambda: snapshot('vehicle_status').get('nav_state') == 3
                 and snapshot('vehicle_local_position').get('z', 0) < -1.5, 90)
        initial_seq = snapshot('mission_result')['seq_current']
        commandline('param', 'set', 'SIM_GPS_USED', '0')
        wait_for('gnss_rejected', lambda: not snapshot('estimator_status_flags').get('cs_gnss_pos'))
        wait_for('mission_progress_without_gnss', lambda: snapshot('mission_result').get('seq_current', -1) > initial_seq
                 and snapshot('vehicle_status').get('nav_state') == 3, sustained=3)
        commandline('param', 'set', 'SIM_GPS_USED', '10')
        wait_for('joint_fusion_recovered', joint_fusion, sustained=3)
        commandline('param', 'set', 'SIM_GPS_USED', '0')
        wait_for('second_gnss_loss', lambda: not snapshot('estimator_status_flags').get('cs_gnss_pos'))
        adapter_pid = int(re.findall(r'\[vio_estimator_odometry_adapter-\d+\]: process started with pid \[(\d+)\]',
                                    args.launch_log.read_text())[-1])
        if b'vio_estimator_odometry_adapter' not in Path(f'/proc/{adapter_pid}/cmdline').read_bytes():
            raise RuntimeError('Adapter PID is not owned by the supplied launch log')
        os.kill(adapter_pid, signal.SIGSTOP)
        try:
            wait_for('both_lost_native_descend', lambda: snapshot('vehicle_status').get('nav_state') == 12
                     and snapshot('vehicle_status').get('failsafe')
                     and not snapshot('vehicle_local_position').get('xy_valid'))
            wait_for('landed_disarmed', lambda: snapshot('vehicle_status').get('arming_state') == 1)
        finally:
            os.kill(adapter_pid, signal.SIGCONT)
        save('result', 'PASS scenario driver; ULog continuity audit required separately')
    except Exception as error:
        save('result', 'FAIL/BLOCKED: ' + str(error))
        raise
    finally:
        if connection:
            connection.close()


if __name__ == '__main__':
    main()
