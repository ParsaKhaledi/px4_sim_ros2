import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from gz_pose import (  # noqa: E402
    ned_from_enu,
    parse_pose_v,
    parse_rtf,
    px4_gt_error_m,
    setpoint_world,
    tilt_deg,
)


POSE = """
pose {
  name: "ground_plane"
  position {
    x: 0
    y: 0
    z: 0
  }
  orientation {
    x: 0
    y: 0
    z: 0
    w: 1
  }
}
pose {
  name: "x500_0"
  position {
    x: -3
    y: -1.6
    z: 2.2
  }
  orientation {
    x: 0
    y: 0
    z: 0
    w: 1
  }
}
"""


def test_parse_picks_x500_0_for_the_plain_model():
    pose = parse_pose_v(POSE, "x500")
    assert pose is not None
    assert pose["name"] == "x500_0"
    assert pose["z"] == 2.2
    assert pose["tilt_deg"] == 0


def test_parse_ignores_a_different_airframe():
    text = POSE.replace('name: "x500_0"', 'name: "x500_depth_0"')
    assert parse_pose_v(text, "x500") is None


def test_ned_round_trip_and_px4_error():
    origin = (-3.0, -1.6, 0.2)
    north, east, down = ned_from_enu(-3.0, -1.4, 2.2, origin)
    assert math.isclose(north, 0.2, abs_tol=1e-9)
    assert math.isclose(east, 0.0, abs_tol=1e-9)
    assert math.isclose(down, -2.0, abs_tol=1e-9)
    world = setpoint_world(north, east, down, origin)
    assert math.isclose(world[0], -3.0, abs_tol=1e-9)
    assert math.isclose(world[1], -1.4, abs_tol=1e-9)
    assert math.isclose(world[2], 2.2, abs_tol=1e-9)
    error = px4_gt_error_m((north, east, down), (-3.0, -1.4, 2.2), origin)
    assert error < 1e-9


def test_tilt_of_a_rolled_body():
    # 90 degree roll about x: quaternion x=sin(45), w=cos(45).
    half = 0.70710678118
    assert tilt_deg(half, 0.0, 0.0, half) > 80


def test_parse_rtf_reads_the_stats_line():
    text = "iterations: 10\nreal_time_factor: 0.42\n"
    assert parse_rtf(text) == 0.42
    assert parse_rtf("no stats here") is None
