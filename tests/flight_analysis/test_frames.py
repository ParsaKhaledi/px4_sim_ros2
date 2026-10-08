"""ENU/FLU to NED/FRD for one known pose, plus the tilt of a pure roll."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from flight_analysis.frames import enu_flu_to_ned_frd  # noqa: E402
from flight_analysis.frames import tilt_rad_from_quaternion  # noqa: E402
from flight_analysis.frames import yaw_from_ned_quaternion  # noqa: E402


class FrameConversionTests(unittest.TestCase):
    """Position and yaw for poses a TUM file could contain."""

    def test_east_north_up_pose_facing_east(self) -> None:
        """ENU (1 m east, 2 m north, 3 m up), identity attitude, faces east.

        That is north=2, east=1, down=-3, and NED yaw +90 deg.
        """

        pose = enu_flu_to_ned_frd(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(pose.north_m, 2.0)
        self.assertAlmostEqual(pose.east_m, 1.0)
        self.assertAlmostEqual(pose.down_m, -3.0)
        self.assertAlmostEqual(pose.yaw_rad(), math.pi / 2, places=6)

    def test_facing_north_is_zero_yaw_in_ned(self) -> None:
        """ENU yaw +90 deg points north, which is yaw 0 in NED."""

        half = math.pi / 4.0
        pose = enu_flu_to_ned_frd(0.0, 0.0, 0.0, 0.0, 0.0, math.sin(half), math.cos(half))
        self.assertAlmostEqual(pose.north_m, 0.0)
        self.assertAlmostEqual(pose.east_m, 0.0)
        self.assertAlmostEqual(pose.down_m, 0.0)
        self.assertAlmostEqual(pose.yaw_rad(), 0.0, places=6)
        self.assertAlmostEqual(pose.qw, 1.0, places=6)

    def test_yaw_tracks_pi_over_2_minus_enu_yaw(self) -> None:
        """A few headings, so the yaw relationship is not a single special case."""

        for enu_yaw in (0.0, math.radians(30), math.pi / 2, math.pi, -math.pi / 2):
            half = enu_yaw / 2.0
            pose = enu_flu_to_ned_frd(0.0, 0.0, 0.0, 0.0, 0.0, math.sin(half), math.cos(half))
            expected = (math.pi / 2.0 - enu_yaw + math.pi) % (2.0 * math.pi) - math.pi
            if math.isclose(expected, -math.pi):
                expected = math.pi
            self.assertAlmostEqual(pose.yaw_rad(), expected, places=6)

    def test_pure_roll_tilt_matches_the_roll_angle(self) -> None:
        """Tilt off vertical for a 20 deg roll is 20 deg."""

        roll = math.radians(20.0)
        tilt = tilt_rad_from_quaternion(math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
        self.assertAlmostEqual(math.degrees(tilt), 20.0, places=5)
        self.assertAlmostEqual(yaw_from_ned_quaternion(1.0, 0.0, 0.0, 0.0), 0.0)


if __name__ == "__main__":
    unittest.main()
