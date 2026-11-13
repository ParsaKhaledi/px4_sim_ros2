"""Frame conversion tests with known values."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from trajectory_eval.frames import (
    geodetic_to_enu,
    gps_fields_to_lla,
    ned_frd_to_enu_flu,
    ned_to_enu_position,
    quat_from_rpy,
    quat_to_rot,
    yaw_from_quat,
)


def test_ned_position_to_enu():
    enu = ned_to_enu_position(np.array([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(enu, [2.0, 1.0, -3.0])


def test_level_body_frd_aligned_with_ned_points_north_in_enu():
    position, quat = ned_frd_to_enu_flu(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(position, [0.0, 0.0, 0.0])
    rotation = quat_to_rot(quat)
    np.testing.assert_allclose(rotation[:, 0], [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(rotation[:, 1], [-1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-9)
    assert abs(yaw_from_quat(quat) - (np.pi / 2.0)) < 1e-9


def test_yaw_only_ned_to_enu():
    q_ned = quat_from_rpy(0.0, 0.0, np.deg2rad(90.0))
    _position, quat = ned_frd_to_enu_flu(np.zeros(3), q_ned)
    rotation = quat_to_rot(quat)
    # 90 deg yaw in NED points the nose east. ENU forward axis is +x.
    np.testing.assert_allclose(rotation[:, 0], [1.0, 0.0, 0.0], atol=1e-9)


def test_geodetic_equator_offsets():
    east = geodetic_to_enu(0.0, 0.001, 0.0, 0.0, 0.0, 0.0)
    # One thousandth of a degree of longitude at the equator.
    np.testing.assert_allclose(east[0], 111.319490793, atol=1e-3)
    np.testing.assert_allclose(east[1], 0.0, atol=1e-6)
    # A point on the ellipsoid sits a fraction of a millimetre below the local tangent plane.
    assert abs(east[2]) < 0.002
    north = geodetic_to_enu(0.001, 0.0, 10.0, 0.0, 0.0, 0.0)
    assert 110.0 < north[1] < 111.0
    np.testing.assert_allclose(north[2], 10.0, atol=1e-3)
    same = geodetic_to_enu(37.0, -122.0, 20.0, 37.0, -122.0, 20.0)
    np.testing.assert_allclose(same, [0.0, 0.0, 0.0], atol=1e-6)


def test_gps_field_layouts():
    class SensorGps:
        latitude_deg = 50.0
        longitude_deg = 19.0
        altitude_msl_m = 12.5

    class Legacy:
        lat = 500000000
        lon = 190000000
        alt = 12500

    assert gps_fields_to_lla(SensorGps()) == (50.0, 19.0, 12.5)
    lat, lon, alt = gps_fields_to_lla(Legacy())
    assert abs(lat - 50.0) < 1e-9
    assert abs(lon - 19.0) < 1e-9
    assert abs(alt - 12.5) < 1e-9
