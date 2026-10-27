#!/usr/bin/env python3
"""Audit recorded PX4 SITL acceptance; never publish data or command a vehicle."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def analyze(path, jump_limit):
    """Check the GNSS-loss/recovery/both-loss sequence in the actual flight log."""
    log = ULog(str(path))
    local = log.get_dataset('vehicle_local_position').data
    flags = log.get_dataset('estimator_status_flags').data
    status = log.get_dataset('vehicle_status').data
    mission = log.get_dataset('mission_result').data
    t = local['timestamp']

    def sample(data, key):
        indices = np.searchsorted(data['timestamp'], t, side='right') - 1
        result = data[key][np.maximum(indices, 0)].copy()
        result[indices < 0] = 0
        return result

    gnss = sample(flags, 'cs_gnss_pos').astype(bool)
    gnss_vel = sample(flags, 'cs_gnss_vel').astype(bool)
    ev = sample(flags, 'cs_ev_vel').astype(bool)
    ev_hgt = sample(flags, 'cs_ev_hgt').astype(bool)
    armed = sample(status, 'arming_state') == 2
    nav = sample(status, 'nav_state')
    failsafe = sample(status, 'failsafe').astype(bool)
    seq = sample(mission, 'seq_current')
    valid = local['xy_valid'].astype(bool) & local['v_xy_valid'].astype(bool)
    both = gnss & gnss_vel & ev & ev_hgt

    def first(mask):
        indices = np.flatnonzero(mask)
        if not len(indices):
            raise ValueError('Required acceptance phase absent from this log')
        return int(indices[0])

    arm = first(armed)
    outage = first(armed & ~gnss & ev)
    recovery = first((t > t[outage]) & armed & both)
    total_loss = first((t > t[recovery]) & armed & ~gnss & ~ev)
    fallback = first((t >= t[total_loss]) & armed & failsafe & ~valid & (nav != 3))
    disarm = first((t > t[fallback]) & ~armed)
    denied = (t >= t[outage]) & (t < t[recovery])
    # Exclude initial global-origin establishment and the final no-aid landing.
    controlled = (t >= t[arm]) & (t < t[total_loss]) & (nav == 3)
    positions = np.column_stack([local[k] for k in ('x', 'y', 'z')])
    velocities = np.column_stack([local[k] for k in ('vx', 'vy', 'vz')])
    dt = np.diff(t.astype(float)) / 1e6
    consecutive = controlled[1:] & controlled[:-1] & (dt > 0) & (dt < 0.2)
    residual = np.linalg.norm(np.diff(positions, axis=0) - velocities[:-1] * dt[:, None], axis=1)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    max_residual = float(residual[consecutive].max())
    resets = {}
    for name in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'):
        changed = np.diff(local[name].astype(int)) != 0
        resets[name] = int(np.sum(changed & consecutive))
    before_arm = both & (t < t[arm])
    preflight_duration = (t[arm] - t[first(before_arm)]) / 1e6
    checks = {
        'gnss_and_vio_before_arm': bool(both[arm] and preflight_duration >= 2.0),
        'mission_continues_without_gnss': bool(np.all(nav[denied] == 3)
                                             and np.all(valid[denied])
                                             and not np.any(failsafe[denied])
                                             and seq[recovery - 1] > seq[outage]),
        'gnss_recovers_with_vio': bool(both[recovery] and nav[recovery] == 3),
        'no_large_position_discontinuity': bool(max_residual <= jump_limit),
        'no_position_or_heading_reset_during_aided_mission': not any(resets.values()),
        # PX4 50dabd8: navigation state 12 = Descend, arming state 1 = Disarmed.
        'native_both_loss_failsafe_and_disarm': bool(nav[fallback] == 12
                                                  and not armed[disarm]),
    }
    return {
        'log': path.name,
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'px4_git': log.msg_info_dict.get('ver_sw', 'unknown'),
        'checks': checks,
        'result': 'PASS' if all(checks.values()) else 'FAIL',
        'phase_time_s': {name: float(t[index] / 1e6) for name, index in {
            'arm': arm, 'gnss_out': outage, 'gnss_back': recovery,
            'both_out': total_loss, 'px4_descend': fallback, 'disarm': disarm,
        }.items()},
        'prearm_joint_fusion_s': float(preflight_duration),
        'gnss_outage_mission_sequences': [int(seq[outage]), int(seq[recovery - 1])],
        'max_position_step_m': float(steps[consecutive].max()),
        'max_position_motion_residual_m': max_residual,
        'jump_limit_m': jump_limit,
        'aided_mission_reset_counts': resets,
        'limitations': ['Single SITL flight; not hardware or long-outage certification',
                        'Manual preflight gate; firmware does not require EV before arming'],
    }


def main():
    """Write reviewable JSON and fail if a phase or acceptance check is missing."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--jump-limit-m', type=float, default=0.5)
    args = parser.parse_args()
    if not np.isfinite(args.jump_limit_m) or args.jump_limit_m <= 0:
        parser.error('jump limit must be positive and finite')
    try:
        result = analyze(args.ulog, args.jump_limit_m)
    except (KeyError, IndexError, ValueError) as error:
        result = {'result': 'INCOMPLETE', 'reason': str(error)}
    rendered = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end='')
    raise SystemExit(0 if result['result'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
