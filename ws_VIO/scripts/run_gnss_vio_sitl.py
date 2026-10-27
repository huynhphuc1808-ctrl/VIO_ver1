#!/usr/bin/env python3
"""Bound one fresh GNSS+VIO native Navigator SITL regression; never use on hardware."""

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

from collect_phase1_sitl import atomic_json, utc_now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=480)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 120 <= args.timeout <= 900:
        parser.error('timeout must be finite and between 120 and 900 seconds')
    workspace = Path(__file__).resolve().parents[1]
    px4 = Path(os.environ['PX4_DIR']).resolve()
    binaries = px4 / 'build/px4_sitl_default/bin'
    if not (binaries / 'px4').is_file():
        parser.error('PX4_DIR must identify the built SITL checkout')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '42'),
               ROS_LOCALHOST_ONLY='1', GZ_PARTITION='uav_vio', GZ_IP='127.0.0.1',
               PX4_PARAM_SDLOG_DIRS_MAX='0', OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1',
               PX4_PARAM_MPC_Z_VEL_MAX_UP='3.0', PX4_PARAM_MPC_Z_V_AUTO_UP='3.0')
    processes = []
    result = dict(success=False, scope='SITL only; native Navigator, no ROS mission backend',
                  started_utc=utc_now())
    result['source_sha256'] = {name: hashlib.sha256((workspace / name).read_bytes()).hexdigest()
        for name in ('scripts/run_gnss_vio_sitl.py', 'scripts/test_gnss_vio_mission.py',
                     'src/uav_vio_sim/config/gnss_vio.params',
                     'src/uav_vio_sim/models/d435i_sensor/model.sdf')}

    def event(kind, **fields):
        row = dict(event=kind, utc=utc_now(), wall=time.monotonic(), **fields)
        with (output / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(row) + '\n')
        print(json.dumps(row), flush=True)

    def launch(name, command):
        stream = (output / (name + '.log')).open('w')
        child = subprocess.Popen(command, cwd=workspace, start_new_session=True,
            env=dict(env, ROS_LOG_DIR=str(output / ('ros_' + name))),
            stdout=stream, stderr=subprocess.STDOUT)
        processes.append((name, child, stream))
        event('process_started', name=name, pid=child.pid, command=command)
        return child

    def latest():
        try:
            return json.loads((output / 'collector/latest.json').read_text())
        except (ValueError, FileNotFoundError):
            return {'topics': {}}

    def message(snapshot, token):
        return next((row['message'] for name, row in snapshot['topics'].items()
            if re.fullmatch(re.escape(token) + r'(?:_v\d+)?', name.rsplit('/', 1)[-1])), {})

    def stationary(snapshot):
        now = time.monotonic()
        for token, ttl in (('/vio/odometry', .5), ('/vio/status', .5),
                           ('/vehicle_local_position', .5), ('/vehicle_status', 2),
                           ('/estimator_status_flags', 5)):
            rows = [row for name, row in snapshot['topics'].items() if token in name]
            if not rows or now - rows[0]['receipt_steady_s'] >= ttl:
                return False
        local = message(snapshot, 'vehicle_local_position')
        flags = message(snapshot, 'estimator_status_flags')
        velocity = message(snapshot, 'odometry').get('twist', {}).get('twist', {}).get('linear', {})
        return (message(snapshot, 'vehicle_status').get('arming_state') == 1 and
                message(snapshot, 'vehicle_land_detected').get('landed') is True and
                all(local.get(k) is True for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'xy_global')) and
                all(flags.get(k) is True for k in ('cs_gnss_pos', 'cs_gnss_vel', 'cs_ev_vel', 'cs_ev_hgt')) and
                snapshot['topics'].get('/vio/status', {}).get('message', {}).get('data') == 'VALID_ODOMETRY' and
                math.sqrt(sum(velocity.get(k, 100) ** 2 for k in ('x', 'y', 'z'))) < .15 and
                math.sqrt(sum(local.get(k, 100) ** 2 for k in ('vx', 'vy', 'vz'))) < .15)

    try:
        launch('collector', [sys.executable, str(workspace / 'scripts/collect_phase1_sitl.py'),
                            '--output', str(output / 'collector'), '--duration', str(args.timeout + 240),
                            '--snapshot-interval', '.1'])
        launch('base', ['ros2', 'launch', 'uav_vio_bringup', 'x500_d435i.launch.py',
                        'world:=kthspacelab', 'gui:=false', 'rviz:=false', 'px4_parameter_profile:=gnss_vio'])
        deadline, since = time.monotonic() + 120, None
        while time.monotonic() < deadline:
            if any(child.poll() is not None for _, child, _ in processes):
                raise RuntimeError('A startup process exited')
            if stationary(latest()):
                since = since or time.monotonic()
                if time.monotonic() - since >= 2:
                    break
            else:
                since = None
            time.sleep(.1)
        else:
            raise RuntimeError('Genuine stationary joint-fusion prearm gate timed out')
        atomic_json(output / 'stationary_prearm.json', latest())
        graph = json.loads((output / 'collector/graph_latest.json').read_text())
        if any(entry.get('publishers') for topic, entry in graph['monitored'].items()
               if '/in/vehicle_command' in topic or '/in/goto_setpoint' in topic):
            raise RuntimeError('Unexpected ROS flight-command publisher in native Navigator regression')
        atomic_json(output / 'flight_graph.json', graph)
        event('stationary_joint_fusion_prearm')
        driver = launch('driver', [sys.executable, str(workspace / 'scripts/test_gnss_vio_mission.py'),
            '--px4-dir', str(px4), '--launch-log', str(output / 'base.log'),
            '--output', str(output / 'driver_results.json'), '--allow-sitl-arm'])
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and driver.poll() is None:
            if any(child.poll() is not None for name, child, _ in processes if name != 'driver'):
                raise RuntimeError('A simulation process exited during the driver')
            time.sleep(.2)
        if driver.poll() is None:
            raise RuntimeError('Native Mission regression exceeded the bounded deadline')
        if driver.returncode:
            raise RuntimeError('Native Mission driver failed; inspect driver_results.json and driver.log')
        time.sleep(5)
        final = latest()
        atomic_json(output / 'final.json', final)
        if message(final, 'vehicle_status').get('arming_state') != 1 or message(final, 'vehicle_land_detected').get('landed') is not True:
            raise RuntimeError('Driver success without native landed/disarmed state')
        result['success'] = True
        event('landed_disarmed_success')
    except BaseException as error:
        result['error'] = repr(error)
        atomic_json(output / 'failure_snapshot.json', latest())
        event('episode_failed', error=repr(error))
        if message(latest(), 'vehicle_status').get('arming_state') == 2:
            # Recovery is explicitly recorded and never makes a failed test pass.
            recovery = subprocess.run([str(binaries / 'px4-commander'), '--instance', '0', 'land'],
                                      capture_output=True, text=True, timeout=5)
            atomic_json(output / 'manual_recovery.json', dict(command='native Land',
                        returncode=recovery.returncode, stdout=recovery.stdout, stderr=recovery.stderr))
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline and message(latest(), 'vehicle_status').get('arming_state') == 2:
                time.sleep(.2)
    finally:
        shutdown = []
        for name, child, stream in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
            shutdown.append(dict(name=name, pid=child.pid, exit_code=child.returncode))
            stream.close()
        log = (output / 'base.log').read_text(errors='replace') if (output / 'base.log').exists() else ''
        paths = re.findall(r'\./log/(\d{4}-\d{2}-\d{2}/[\d_]+\.ulg)', log)
        if paths:
            source = px4 / 'build/px4_sitl_default/rootfs/0/log' / paths[-1]
            if source.exists():
                shutil.copy2(source, output / 'flight.ulg')
                result['ulog_sha256'] = hashlib.sha256((output / 'flight.ulg').read_bytes()).hexdigest()
        result.update(ended_utc=utc_now(), shutdown=shutdown)
        atomic_json(output / 'result.json', result)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
