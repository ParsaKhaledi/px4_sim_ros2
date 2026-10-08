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
from flight_analysis.frames import spawn_enu_to_world_enu  # noqa: E402
from flight_analysis.frames import tilt_rad_from_quaternion  # noqa: E402
from flight_analysis.frames import world_enu_to_spawn_enu  # noqa: E402
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

    def test_yawed_spawn_round_trip(self) -> None:
        """A pose in the spawn frame survives a trip out to world ENU and back.

        Spawn yaw is +90 deg, so one metre along the spawn x axis is one
        metre north in the Gazebo world, not one metre east. Forgetting
        the yaw would leave the point at world (1, 0) and the round trip
        would not recover (1, 0) in the spawn frame.
        """

        spawn = (10.0, -4.0, 0.5)
        yaw = math.pi / 2.0
        local_yaw = 0.4
        half = local_yaw / 2.0
        local = (1.0, -0.25, 2.0, 0.0, 0.0, math.sin(half), math.cos(half))
        world = spawn_enu_to_world_enu(*local, spawn, yaw)
        self.assertAlmostEqual(world[0], spawn[0] + 0.25, places=6)
        self.assertAlmostEqual(world[1], spawn[1] + 1.0, places=6)
        self.assertAlmostEqual(world[2], spawn[2] + 2.0, places=6)
        back = world_enu_to_spawn_enu(*world, spawn, yaw)
        for got, expected in zip(back, local, strict=True):
            self.assertAlmostEqual(got, expected, places=6)
        ned = enu_flu_to_ned_frd(*back)
        direct = enu_flu_to_ned_frd(*local)
        self.assertAlmostEqual(ned.north_m, direct.north_m, places=6)
        self.assertAlmostEqual(ned.east_m, direct.east_m, places=6)
        self.assertAlmostEqual(ned.down_m, direct.down_m, places=6)
        self.assertAlmostEqual(ned.yaw_rad(), direct.yaw_rad(), places=6)


if __name__ == "__main__":
    unittest.main()
