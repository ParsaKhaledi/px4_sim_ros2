"""Out-and-back segmentation, TUM truth preferred over the log, and the CLI."""

from __future__ import annotations

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
for entry in (str(ROOT), str(HERE)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

from flight_analysis.analyze import analyze_flight  # noqa: E402
from flight_analysis.cli import main  # noqa: E402
from flight_analysis.log import LogLoader  # noqa: E402
from flight_analysis.log import command_track  # noqa: E402
from flight_analysis.segment import segment_command  # noqa: E402
from synthetic import fixture_values  # noqa: E402
from synthetic import flight_log  # noqa: E402
from synthetic import level_quaternion  # noqa: E402
from synthetic import load_fixture_limits  # noqa: E402
from synthetic import microseconds  # noqa: E402
from synthetic import times  # noqa: E402


def _mission(limits, logged_truth: bool = False, truth_offset_m: float = 0.0):
    """Take off, hold, go out, yaw 180, come back, land. Estimate tracks the command."""

    phase = limits.settle_hold_s * 2
    dt = 0.05
    time_s = times(0.0, phase * 6, dt)
    north = np.zeros_like(time_s)
    down = np.zeros_like(time_s)
    yaw = np.zeros_like(time_s)
    north[(time_s >= 2 * phase) & (time_s < 4 * phase)] = 0.3
    down[(time_s >= phase) & (time_s < 5 * phase)] = -2.0
    yaw[time_s >= 3 * phase] = math.pi
    # The return leg brings north back to zero. Yaw stays at pi through the land.
    stamp = microseconds(time_s)
    zeros = np.zeros_like(time_s)
    topics = {
        "trajectory_setpoint": {
            "timestamp": stamp,
            "position[0]": north,
            "position[1]": zeros,
            "position[2]": down,
            "yaw": yaw,
        },
        "vehicle_local_position": {
            "timestamp": stamp,
            "x": north,
            "y": zeros,
            "z": down,
            "heading": yaw,
        },
    }
    attitude = level_quaternion(time_s.shape[0])
    attitude["timestamp"] = stamp
    topics["vehicle_attitude"] = attitude
    topics["actuator_motors"] = {
        "timestamp": stamp,
        "control[0]": np.full(time_s.shape, 0.4),
    }
    if logged_truth:
        topics["vehicle_local_position_groundtruth"] = {
            "timestamp": stamp,
            "x": north,
            "y": zeros,
            "z": down - truth_offset_m,
            "heading": yaw,
        }
    return flight_log(topics, {"MPC_TILTMAX_AIR": 45.0}), time_s, north, down


class MissionTests(unittest.TestCase):
    """The scripted out-and-back, and TUM preferred over logged truth."""

    def test_legs_match_the_out_and_back(self) -> None:
        """Hold, climb, forward, yaw, forward, land."""

        limits = load_fixture_limits()
        log, _time_s, _north, _down = _mission(limits)
        kinds = [leg.kind for leg in segment_command(command_track(log))]
        self.assertEqual(kinds, ["hold", "takeoff", "translate", "yaw", "translate", "land"])
        report = analyze_flight(log, limits)
        self.assertTrue(report["passed"], [item for item in report["checks"] if not item["passed"]])
        self.assertAlmostEqual(report["flight"]["return_distance_m"], 0.0, places=3)

    def test_tum_truth_is_preferred_over_the_log(self) -> None:
        """Estimation error follows the TUM file when both sources exist."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits, logged_truth=True)
        bias = limits.height_tolerance_m * 2
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "gt.tum"
            lines = []
            for stamp, n, d in zip(time_s, north, down, strict=True):
                # ENU: east, north, up. The bias is extra up, so NED down is more negative.
                lines.append(f"{stamp:.3f} 0 {n:.4f} {-d + bias:.4f} 0 0 0 1")
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            report = analyze_flight(
                log,
                limits,
                tum_path=path,
                spawn_xyz=(0.0, 0.0, 0.0),
                spawn_yaw=0.0,
                clock_offset_s=0.0,
            )
        self.assertEqual(report["ground_truth"]["source"], "tum")
        self.assertIn("not used", report["ground_truth"]["detail"])
        self.assertAlmostEqual(report["ground_truth"]["time_offset_s"], 0.0, delta=0.05)
        vertical = report["flight"]["estimation_error"]["vertical_peak_m"]
        self.assertAlmostEqual(vertical, bias, delta=0.02)


class _MemoryLoader(LogLoader):
    """Loader double that returns one prepared FlightLog."""

    def __init__(self, log) -> None:
        """Remember the log the command should grade."""

        self.log = log

    def load(self, path: Path):
        """Ignore the path and return the prepared log."""

        del path
        return self.log


class CliTests(unittest.TestCase):
    """The command writes metrics.json and the four plots."""

    def test_cli_writes_a_passing_report(self) -> None:
        """A tracking estimate with the fixture limits passes and writes files."""

        limits = load_fixture_limits()
        log, _time_s, _north, _down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            ulg = Path(folder) / "demo.ulg"
            ulg.write_bytes(b"")
            output = Path(folder) / "out"
            with patch.dict(os.environ, fixture_values(), clear=False):
                code = main(
                    [str(ulg), "--output", str(output), "--run-id", "demo"],
                    loader=_MemoryLoader(log),
                )
            self.assertEqual(code, 0)
            self.assertTrue((output / "metrics.json").is_file())
            for name in ("position.png", "yaw.png", "tilt.png", "errors.png"):
                path = output / name
                self.assertTrue(path.is_file(), name)
                self.assertGreater(path.stat().st_size, 1000)
