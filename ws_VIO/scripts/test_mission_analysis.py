#!/usr/bin/env python3
"""Read-only circle analyzer regressions; no ROS initialization or commands."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import analyze_vio_circle_log as analyzer


class CircleLogFixture:
    """Small complete flight trace with a normal-channel bootstrap selection."""

    def __init__(self, executor_id: int, local_mode_id: int):
        source = 1000 + executor_id
        owned_mode_id = 23 + (local_mode_id - 23 + 1) % 8
        commands = {
            "timestamp": [0, 10, 10, 15, 20, 220, 230],
            "command": [100001, 400, 22, 100001, 34, 100001, 100001],
            "param1": [owned_mode_id, 1, math.nan, local_mode_id, 2, local_mode_id, 18],
            "param2": [0, math.nan, math.nan, 0, 1, 0, 0],
            "source_component": [1, source, source, source, source, source, source],
        }
        angles = [i * math.pi / 8 for i in range(18)]
        self.data = {
            "vehicle_command": commands,
            "vehicle_local_position": {
                "timestamp": [0, 10, 20] + [30 + 10 * i for i in range(len(angles))],
                "x": [0, 0, 0] + [2 * math.cos(angle) for angle in angles],
                "y": [0, 0, 0] + [2 * math.sin(angle) for angle in angles],
                "z": [0, 0, -2] + [-2] * len(angles),
            },
            "vehicle_status": {
                "timestamp": [0, 10, 15, 30, 230, 250],
                "arming_state": [1, 2, 2, 2, 2, 1],
                "nav_state": [owned_mode_id, 17, local_mode_id, 21, 18, owned_mode_id],
                "failsafe": [False] * 6,
            },
            "vehicle_land_detected": {"timestamp": [0, 30, 250], "landed": [True, False, True]},
            "estimator_status_flags": {
                "timestamp": [30, 100, 200],
                "cs_ev_pos": [True] * 3, "cs_ev_vel": [True] * 3, "cs_ev_hgt": [True] * 3,
                "cs_ev_yaw": [False] * 3, "cs_gnss_pos": [False] * 3, "cs_gnss_vel": [False] * 3,
            },
        }
        self.initial_parameters = {
            "SYS_HAS_GPS": 0, "SIM_GZ_EN_GPS": 0, "SIM_GPS_USED": 0,
            "SENS_EN_GPSSIM": 0, "EKF2_GPS_CTRL": 0, "EKF2_EV_CTRL": 7,
            "COM_FAIL_ACT_T": 5, "COM_POS_FS_EPH": 5, "COM_VEL_FS_EVH": 1,
            "COM_LOW_BAT_ACT": 3, "COM_POS_LOW_ACT": 3, "NAV_DLL_ACT": 0,
            "NAV_RCL_ACT": 2, "GF_ACTION": 2,
        }

    def get_dataset(self, name: str):
        return types.SimpleNamespace(data=self.data[name])


class MissionAnalysisTest(unittest.TestCase):
    def analyze_fixture(self, executor_id=1, local_mode_id=23, *, wrong_source=False, orbit_index=0):
        with tempfile.TemporaryDirectory(prefix="vio_circle_analysis_") as directory:
            path = Path(directory) / "fixture.ulg"
            path.write_bytes(b"Synthetic trace; ULog decoding supplied by the fixture.")
            args = analyzer.parse_arguments([
                str(path), "--executor-id", str(executor_id), "--local-mode-id", str(local_mode_id),
                "--orbit-index", str(orbit_index),
            ])
            fixture = CircleLogFixture(executor_id, local_mode_id)
            if wrong_source:
                fixture.data["vehicle_command"]["source_component"][2] = 1
            module = types.SimpleNamespace(ULog=lambda _: fixture)
            with patch.dict(sys.modules, {"pyulog": module}):
                return analyzer.analyze(args)

    def test_dynamic_ids_find_interval_and_exclude_bootstrap(self):
        report = self.analyze_fixture(executor_id=3, local_mode_id=28)
        self.assertTrue(report["valid"], report["failed_checks"])
        self.assertEqual(report["configuration"]["executor_id"], 3)
        self.assertEqual(report["configuration"]["local_mode_id"], 28)
        self.assertEqual(report["configuration"]["mission_source_component"], 1003)
        self.assertEqual(report["metrics"]["orbit_end_timestamp_us"], 220)
        commands = report["checks"]["mission_command_source"]["observed"]
        self.assertEqual(commands["schedule"], [1003, 1003])
        self.assertTrue(all(command["source_component"] == 1003 for command in report["mission_commands"]))

    def test_wrong_flight_command_source_fails_even_with_bootstrap_excluded(self):
        report = self.analyze_fixture(executor_id=3, local_mode_id=28, wrong_source=True)
        self.assertFalse(report["valid"])
        self.assertEqual(report["failed_checks"], ["mission_command_source"])
        self.assertEqual(report["checks"]["mission_command_source"]["observed"]["takeoff"], [1])

    def test_legacy_defaults_preserve_source_and_schedule_contract(self):
        args = analyzer.parse_arguments(["legacy.ulg"])
        self.assertEqual((args.executor_id, args.local_mode_id), (1, 23))
        report = self.analyze_fixture()
        self.assertTrue(report["valid"], report["failed_checks"])
        self.assertEqual(report["configuration"]["mission_source_component"], 1001)

    def test_missing_second_orbit_cannot_reuse_first_orbit_proof(self):
        with self.assertRaisesRegex(RuntimeError, "requested Orbit index 1, but ULog contains 1"):
            self.analyze_fixture(orbit_index=1)
        for value in ("-1", "1024", "1.5", "nan"):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    analyzer.parse_arguments(["fixture.ulg", "--orbit-index", value])
                self.assertEqual(result.exception.code, 2)

    def test_id_bounds_and_integer_types(self):
        for executor_id in (1, 5):
            for local_mode_id in (23, 30):
                with self.subTest(executor_id=executor_id, local_mode_id=local_mode_id):
                    args = analyzer.parse_arguments([
                        "fixture.ulg", "--executor-id", str(executor_id), "--local-mode-id", str(local_mode_id),
                    ])
                    self.assertEqual((args.executor_id, args.local_mode_id), (executor_id, local_mode_id))
        invalid = {
            "--executor-id": ("0", "6", "-1", "99999999999999999999", "1.5", "1.0", "nan", "inf"),
            "--local-mode-id": ("22", "31", "-1", "99999999999999999999", "23.5", "23.0", "nan", "inf"),
        }
        for flag, values in invalid.items():
            for value in values:
                with self.subTest(flag=flag, value=value), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as result:
                        analyzer.parse_arguments(["fixture.ulg", flag, value])
                    self.assertEqual(result.exception.code, 2)

    def test_nonfinite_or_nonpositive_measurements_rejected_before_ulog_loading(self):
        for flag in ("--radius", "--altitude", "--speed", "--laps", "--radial-band"):
            for value in ("nan", "inf", "-inf", "1e999", "0", "-1"):
                with self.subTest(flag=flag, value=value), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as result:
                        analyzer.parse_arguments(["fixture.ulg", flag, value])
                    self.assertEqual(result.exception.code, 2)

    def test_stored_legacy_ulog_matches_baseline_when_available(self):
        baseline_path = Path(__file__).resolve().parents[1] / "docs/validation/vio_circle_sitl.json"
        baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
        path = Path(baseline["input"])
        if not path.is_file() or importlib.util.find_spec("pyulog") is None:
            self.skipTest("stored legacy ULog and pyulog are needed for the physical baseline regression")
        args = analyzer.parse_arguments([str(path), "--expected-sha256", baseline["input_sha256"]])
        report = analyzer.analyze(args)
        self.assertTrue(report["valid"], report["failed_checks"])
        self.assertEqual(report["input_sha256"], baseline["input_sha256"])
        self.assertEqual(report["checks"], baseline["checks"])
        self.assertEqual(report["metrics"], baseline["metrics"])


if __name__ == "__main__":
    unittest.main()
