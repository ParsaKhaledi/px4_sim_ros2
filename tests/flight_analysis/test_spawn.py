"""spawn.json, CLI overrides, a missing pose, and the climb-edge clock."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
for entry in (str(ROOT), str(HERE)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

from flight_analysis.analyze import analyze_flight  # noqa: E402
from flight_analysis.spawn import SpawnError  # noqa: E402
from synthetic import load_fixture_limits  # noqa: E402
from test_analyze import _mission  # noqa: E402


def _write_tum(path: Path, time_s, north, down, time_shift_s: float = 0.0) -> None:
    """Write world-ENU poses that match a NED track when the spawn is the origin."""

    lines = []
    for stamp, n, d in zip(time_s, north, down, strict=True):
        lines.append(f"{stamp + time_shift_s:.3f} 0 {n:.4f} {-d:.4f} 0 0 0 1")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class SpawnFileTests(unittest.TestCase):
    """Where the spawn pose and the clock offset come from."""

    def test_spawn_json_beside_the_log_is_used(self) -> None:
        """The file next to the .ulg supplies the pose and px4_offset_s."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down, time_shift_s=4.0)
            (root / "spawn.json").write_text(
                json.dumps(
                    {
                        "spawn_xyz": [0.0, 0.0, 0.0],
                        "spawn_yaw": 0.0,
                        "px4_offset_s": 4.0,
                    }
                ),
                encoding="utf-8",
            )
            warnings: list[str] = []
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                warn=warnings.append,
            )
        clock = report["ground_truth"]["clock"]
        spawn = report["ground_truth"]["spawn"]
        self.assertEqual(spawn["xyz_source"], "spawn_json")
        self.assertEqual(spawn["yaw_source"], "spawn_json")
        self.assertEqual(spawn["xyz_m"], [0.0, 0.0, 0.0])
        self.assertEqual(spawn["yaw_rad"], 0.0)
        self.assertEqual(clock["source"], "spawn_json")
        self.assertAlmostEqual(clock["px4_offset_s"], 4.0)
        self.assertAlmostEqual(clock["applied_offset_s"], -4.0)
        self.assertEqual(warnings, [])
        self.assertAlmostEqual(report["flight"]["estimation_error"]["horizontal_peak_m"], 0.0, delta=0.02)

    def test_cli_overrides_the_file(self) -> None:
        """Manual pose and clock values win, and the file's offset is not estimated."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down, time_shift_s=4.0)
            (root / "spawn.json").write_text(
                json.dumps(
                    {
                        "spawn_xyz": [9.0, 8.0, 7.0],
                        "spawn_yaw": 1.2,
                        "px4_offset_s": 0.0,
                    }
                ),
                encoding="utf-8",
            )
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                spawn_xyz=(0.0, 0.0, 0.0),
                spawn_yaw=0.0,
                clock_offset_s=4.0,
            )
        clock = report["ground_truth"]["clock"]
        spawn = report["ground_truth"]["spawn"]
        self.assertEqual(spawn["xyz_source"], "cli")
        self.assertEqual(spawn["yaw_source"], "cli")
        self.assertEqual(spawn["xyz_m"], [0.0, 0.0, 0.0])
        self.assertEqual(spawn["yaw_rad"], 0.0)
        self.assertEqual(clock["source"], "cli")
        self.assertAlmostEqual(clock["px4_offset_s"], 4.0)
        self.assertAlmostEqual(clock["applied_offset_s"], -4.0)
        self.assertAlmostEqual(report["flight"]["estimation_error"]["horizontal_peak_m"], 0.0, delta=0.02)

    def test_missing_spawn_is_an_error(self) -> None:
        """No file and no manual pose stops the run. The origin is not assumed."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down)
            with self.assertRaises(SpawnError) as caught:
                analyze_flight(log, limits, tum_path=tum, log_path=ulg)
        message = str(caught.exception)
        self.assertIn("identity", message)
        self.assertIn("spawn_xyz", message)
        self.assertIn("spawn_yaw", message)

    def test_missing_clock_uses_the_climb_edge(self) -> None:
        """A pose without px4_offset_s keeps the estimate and records that source."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down, time_shift_s=1.5)
            (root / "spawn.json").write_text(
                json.dumps({"spawn_xyz": [0.0, 0.0, 0.0], "spawn_yaw": 0.0}),
                encoding="utf-8",
            )
            warnings: list[str] = []
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                warn=warnings.append,
            )
        clock = report["ground_truth"]["clock"]
        self.assertEqual(clock["source"], "climb_edge_estimate")
        self.assertAlmostEqual(clock["applied_offset_s"], -1.5, delta=0.05)
        self.assertAlmostEqual(clock["px4_offset_s"], 1.5, delta=0.05)
        self.assertTrue(any("climb-edge" in item for item in warnings))
        self.assertEqual(report["ground_truth"]["spawn"]["xyz_source"], "spawn_json")

    def test_spawn_json_flag_points_at_another_file(self) -> None:
        """``--spawn-json`` is used when the log directory has no spawn.json."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down)
            custom = root / "elsewhere" / "pose.json"
            custom.parent.mkdir()
            custom.write_text(
                json.dumps(
                    {"spawn_xyz": [0.0, 0.0, 0.0], "spawn_yaw": 0.0, "px4_offset_s": 0.0}
                ),
                encoding="utf-8",
            )
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                spawn_json=custom,
            )
        self.assertEqual(report["ground_truth"]["spawn"]["xyz_source"], "spawn_json")
        self.assertEqual(report["ground_truth"]["clock"]["source"], "spawn_json")
        self.assertAlmostEqual(report["ground_truth"]["clock"]["px4_offset_s"], 0.0)

    def test_null_offset_uses_the_climb_and_keeps_the_reason(self) -> None:
        """JSON null is not a clock. The warning quotes px4_offset_reason."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        reason = "PX4 was not publishing vehicle_odometry"
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down, time_shift_s=1.5)
            (root / "spawn.json").write_text(
                json.dumps(
                    {
                        "spawn_xyz": [0.0, 0.0, 0.0],
                        "spawn_yaw": 0.0,
                        "px4_offset_s": None,
                        "px4_offset_reason": reason,
                    }
                ),
                encoding="utf-8",
            )
            warnings: list[str] = []
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                warn=warnings.append,
            )
        clock = report["ground_truth"]["clock"]
        self.assertEqual(clock["source"], "climb_edge_estimate")
        self.assertAlmostEqual(clock["px4_offset_s"], 1.5, delta=0.05)
        self.assertEqual(clock["px4_offset_reason"], reason)
        self.assertTrue(any(reason in item and "climb-edge" in item for item in warnings))

    def test_wide_spread_and_few_samples_warn(self) -> None:
        """A spread over 0.02 s or fewer than 50 samples is a warning, not a failure."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down)
            (root / "spawn.json").write_text(
                json.dumps(
                    {
                        "spawn_xyz": [0.0, 0.0, 0.0],
                        "spawn_yaw": 0.0,
                        "px4_offset_s": 0.0,
                        "px4_offset_spread_s": 0.05,
                        "px4_offset_samples": 12,
                    }
                ),
                encoding="utf-8",
            )
            warnings: list[str] = []
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                warn=warnings.append,
            )
        clock = report["ground_truth"]["clock"]
        self.assertEqual(clock["source"], "spawn_json")
        self.assertAlmostEqual(clock["px4_offset_s"], 0.0)
        self.assertAlmostEqual(clock["px4_offset_spread_s"], 0.05)
        self.assertEqual(clock["px4_offset_samples"], 12)
        self.assertTrue(any("px4_offset_spread_s" in item and "0.02" in item for item in warnings))
        self.assertTrue(any("px4_offset_samples" in item and "50" in item for item in warnings))
        self.assertFalse(any("climb-edge" in item for item in warnings))

    def test_spread_at_the_threshold_does_not_warn(self) -> None:
        """0.02 s and 50 samples are inside the recorder's accepted band."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down)
            (root / "spawn.json").write_text(
                json.dumps(
                    {
                        "spawn_xyz": [0.0, 0.0, 0.0],
                        "spawn_yaw": 0.0,
                        "px4_offset_s": 0.0,
                        "px4_offset_spread_s": 0.02,
                        "px4_offset_samples": 50,
                    }
                ),
                encoding="utf-8",
            )
            warnings: list[str] = []
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                warn=warnings.append,
            )
        self.assertEqual(warnings, [])
        self.assertEqual(report["ground_truth"]["clock"]["px4_offset_samples"], 50)

    def test_unknown_spawn_keys_are_ignored(self) -> None:
        """Extra recorder fields, and near-miss clock names, do not change the pose."""

        limits = load_fixture_limits()
        log, time_s, north, down = _mission(limits)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ulg = root / "flight.ulg"
            ulg.write_bytes(b"")
            tum = root / "gt.tum"
            _write_tum(tum, time_s, north, down)
            (root / "spawn.json").write_text(
                json.dumps(
                    {
                        "spawn_xyz": [0.0, 0.0, 0.0],
                        "spawn_yaw": 0.0,
                        "px4_offset_s": 0.0,
                        "px4_offset_spread": 0.9,
                        "px4_offset_sample_count": 3,
                        "recorder": "trajectory_eval",
                        "note": "ignored",
                    }
                ),
                encoding="utf-8",
            )
            warnings: list[str] = []
            report = analyze_flight(
                log,
                limits,
                tum_path=tum,
                log_path=ulg,
                warn=warnings.append,
            )
        clock = report["ground_truth"]["clock"]
        self.assertEqual(clock["source"], "spawn_json")
        self.assertAlmostEqual(clock["px4_offset_s"], 0.0)
        self.assertNotIn("px4_offset_spread_s", clock)
        self.assertNotIn("px4_offset_samples", clock)
        self.assertEqual(warnings, [])
        self.assertAlmostEqual(report["flight"]["estimation_error"]["horizontal_peak_m"], 0.0, delta=0.02)


if __name__ == "__main__":
    unittest.main()
