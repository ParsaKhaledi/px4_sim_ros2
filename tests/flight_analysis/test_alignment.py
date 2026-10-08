"""TUM clock offset from the climb, and a clear failure when overlap is short."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from flight_analysis.log import Track  # noqa: E402
from flight_analysis.tum import TimeAlignmentError  # noqa: E402
from flight_analysis.tum import align_tum_to_ulog  # noqa: E402
from flight_analysis.tum import load_tum  # noqa: E402


def _bump(time_s: np.ndarray, center: float) -> np.ndarray:
    """Up-velocity shaped like a takeoff, metres per second."""

    return np.exp(-0.5 * ((time_s - center) / 0.4) ** 2)


def _track(time_s: np.ndarray, vz_up: np.ndarray, height: float = 0.0) -> Track:
    """A track whose logged down-velocity is the opposite of ``vz_up``."""

    return Track(
        time_s=time_s,
        north_m=np.zeros_like(time_s),
        east_m=np.zeros_like(time_s),
        down_m=np.full(time_s.shape, -height),
        yaw_rad=np.zeros_like(time_s),
        vd_m_s=-vz_up,
    )


class AlignmentTests(unittest.TestCase):
    """Offset sign, an already-matching clock, and a short file."""

    def test_shifted_climb_recovers_the_offset(self) -> None:
        """A climb 2.5 s later in the TUM clock needs offset -2.5 s."""

        ulog_time = np.arange(0.0, 12.0, 0.02)
        tum_time = ulog_time + 2.5
        alignment = align_tum_to_ulog(
            _track(ulog_time, _bump(ulog_time, 3.0)),
            _track(tum_time, _bump(tum_time, 5.5)),
        )
        self.assertEqual(alignment.method, "vertical_velocity")
        self.assertAlmostEqual(alignment.offset_s, -2.5, delta=0.05)
        self.assertGreater(alignment.correlation, 0.9)
        self.assertGreaterEqual(alignment.overlap_s, 5.0)

    def test_matching_clocks_stay_at_zero(self) -> None:
        """Same timestamps are reported as already aligned."""

        time_s = np.arange(0.0, 12.0, 0.02)
        velocity = _bump(time_s, 3.0)
        alignment = align_tum_to_ulog(_track(time_s, velocity), _track(time_s, velocity))
        self.assertEqual(alignment.method, "already_aligned")
        self.assertAlmostEqual(alignment.offset_s, 0.0)
        self.assertGreater(alignment.correlation, 0.9)

    def test_short_overlap_fails(self) -> None:
        """A ground-truth snippet shorter than the minimum overlap is rejected."""

        ulog_time = np.arange(0.0, 12.0, 0.02)
        tum_time = np.arange(0.0, 0.3, 0.02)
        with self.assertRaises(TimeAlignmentError) as caught:
            align_tum_to_ulog(
                _track(ulog_time, _bump(ulog_time, 3.0)),
                _track(tum_time, _bump(tum_time, 0.1)),
            )
        self.assertIn("overlap", str(caught.exception).lower())

    def test_tum_file_converts_position(self) -> None:
        """One TUM line becomes the NED position of that ENU pose."""

        import tempfile

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "gt.tum"
            path.write_text(
                "# comment\n0.0 1.0 2.0 3.0 0 0 0 1\n0.1 1.0 2.0 3.0 0 0 0 1\n",
                encoding="utf-8",
            )
            track = load_tum(path)
        self.assertAlmostEqual(float(track.north_m[0]), 2.0)
        self.assertAlmostEqual(float(track.east_m[0]), 1.0)
        self.assertAlmostEqual(float(track.down_m[0]), -3.0)


if __name__ == "__main__":
    unittest.main()
