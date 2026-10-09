"""Frame conversions used by the simulation trajectory tools.

Gazebo and ROS use an ENU world (x east, y north, z up) and a FLU body
(x forward, y left, z up). PX4 uses an NED world (x north, y east, z down)
and an FRD body (x forward, y right, z down).

Quaternion convention is Hamiltonian (w, x, y, z). A quaternion rotates a
body-frame vector into the parent frame: ``v_parent = R(q) * v_body``.
"""

from __future__ import annotations

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return a unit quaternion. Input is (w, x, y, z)."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a * b. Both quaternions are (w, x, y, z)."""
    aw, ax, ay, az = quat_normalize(a)
    bw, bx, by, bz = quat_normalize(b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX intrinsic rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = np.cos(half_roll), np.sin(half_roll)
    cp, sp = np.cos(half_pitch), np.sin(half_pitch)
    cy, sy = np.cos(half_yaw), np.sin(half_yaw)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Rotation matrix for a Hamiltonian (w, x, y, z) quaternion."""
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_from_quat(q: np.ndarray) -> float:
    """Yaw about Z for a Z-up quaternion (radians)."""
    w, x, y, z = quat_normalize(q)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quat_xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    """ROS ``(x, y, z, w)`` to the ``(w, x, y, z)`` order used in the math."""
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z], dtype=float)


def quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    """``(w, x, y, z)`` to the ROS message order ``(x, y, z, w)``."""
    w, x, y, z = q_wxyz
    return np.array([x, y, z, w], dtype=float)


def wrap_pi(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


# Static rotations from px4_ros_com frame_transforms.
# NED_ENU_Q = Rz(pi/2) * Rx(pi). AIRCRAFT_BASELINK_Q = Rx(pi).
NED_ENU_Q = quat_from_rpy(np.pi, 0.0, np.pi / 2.0)
AIRCRAFT_BASELINK_Q = quat_from_rpy(np.pi, 0.0, 0.0)


def ned_to_enu_position(position_ned: np.ndarray) -> np.ndarray:
    """Convert a north-east-down position to east-north-up."""
    north, east, down = np.asarray(position_ned, dtype=float)
    return np.array([east, north, -down], dtype=float)


def ned_frd_to_enu_flu(position_ned: np.ndarray, q_ned_wxyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert a PX4 local pose to an ENU position and FLU quaternion.

    ``q_ned_wxyz`` is the body-FRD attitude in the NED frame (w, x, y, z),
    matching ``px4_msgs/VehicleOdometry.q``.
    """
    position_enu = ned_to_enu_position(position_ned)
    q_enu = quat_mul(quat_mul(NED_ENU_Q, q_ned_wxyz), AIRCRAFT_BASELINK_Q)
    return position_enu, quat_normalize(q_enu)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """WGS84 geodetic coordinates to ECEF metres."""
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    normal = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (normal + alt_m) * cos_lat * cos_lon
    y = (normal + alt_m) * cos_lat * sin_lon
    z = (normal * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return np.array([x, y, z], dtype=float)


def ecef_to_enu(ecef: np.ndarray, lat0_deg: float, lon0_deg: float, alt0_m: float) -> np.ndarray:
    """ECEF metres to local ENU metres around a geodetic reference."""
    origin = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    dx, dy, dz = np.asarray(ecef, dtype=float) - origin
    lat = np.deg2rad(lat0_deg)
    lon = np.deg2rad(lon0_deg)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return np.array([east, north, up], dtype=float)


def geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float,
                    lat0_deg: float, lon0_deg: float, alt0_m: float) -> np.ndarray:
    """WGS84 latitude/longitude/altitude to local ENU metres."""
    return ecef_to_enu(geodetic_to_ecef(lat_deg, lon_deg, alt_m), lat0_deg, lon0_deg, alt0_m)


def gps_fields_to_lla(msg) -> tuple[float, float, float] | None:
    """Read lat, lon, alt from a PX4 SensorGps or legacy VehicleGpsPosition."""
    if hasattr(msg, "latitude_deg") and hasattr(msg, "longitude_deg"):
        alt = float(getattr(msg, "altitude_msl_m", 0.0))
        return float(msg.latitude_deg), float(msg.longitude_deg), alt
    if hasattr(msg, "lat") and hasattr(msg, "lon"):
        alt_mm = float(getattr(msg, "alt", 0.0))
        return float(msg.lat) * 1e-7, float(msg.lon) * 1e-7, alt_mm * 1e-3
    return None
