"""FRD/NED to FLU/ENU conversions for the PX4 IMU relay.

PX4 body axes are forward-right-down. ROS body axes are forward-left-up
(REP-103). PX4 attitude is the rotation from the FRD body to the NED world.
``sensor_msgs/Imu`` orientation is the rotation from the FLU body to the ENU
world. The quaternion formula matches ``trajectory_eval.frames`` and
px4_ros_com: ``q_enu = NED_ENU_Q * q_ned * Rx(pi)``.

Angular velocity and linear acceleration are body vectors, so only the axis
map is applied. PX4's accelerometer is specific force. At rest in FRD that is
about ``(0, 0, -g)``; in FLU it is about ``(0, 0, +g)``.
"""

from __future__ import annotations

import math

import numpy as np

G = 9.80665


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Unit quaternion. Input is ``(w, x, y, z)``. A zero quaternion becomes identity."""
    q = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product. Both quaternions are ``(w, x, y, z)``."""
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
    """ZYX intrinsic rotation ``R = Rz(yaw) * Ry(pitch) * Rx(roll)``."""
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix for a ``(w, x, y, z)`` quaternion."""
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# NED_ENU_Q = Rz(pi/2) * Rx(pi). AIRCRAFT_BASELINK_Q = Rx(pi).
NED_ENU_Q = quat_from_rpy(math.pi, 0.0, math.pi / 2.0)
AIRCRAFT_BASELINK_Q = quat_from_rpy(math.pi, 0.0, 0.0)


def frd_to_flu(vector) -> tuple[float, float, float]:
    """Map a forward-right-down vector to forward-left-up."""
    x, y, z = (float(part) for part in vector)
    return (x, -y, -z)


def flu_enu_quaternion(q_frd_ned_wxyz) -> np.ndarray:
    """PX4 FRD-to-NED quaternion to a FLU-to-ENU quaternion, ``(w, x, y, z)``."""
    q_ned = np.asarray(q_frd_ned_wxyz, dtype=float)
    q_enu = quat_mul(quat_mul(NED_ENU_Q, q_ned), AIRCRAFT_BASELINK_Q)
    return quat_normalize(q_enu)


def flu_enu_quaternion_xyzw(q_frd_ned_wxyz) -> tuple[float, float, float, float]:
    """Same rotation in the ``(x, y, z, w)`` order ROS messages use."""
    w, x, y, z = flu_enu_quaternion(q_frd_ned_wxyz)
    return (float(x), float(y), float(z), float(w))


def gravity_quaternion_xyzw(q_frd_ned_wxyz) -> tuple[float, float, float, float]:
    """Roll and pitch of the ENU/FLU attitude, with yaw set to zero.

    The full NED/FRD to ENU/FLU conversion runs first. Yaw is then dropped so
    the quaternion carries gravity only. RTAB-Map is also told to ignore yaw
    by a large yaw variance on the IMU message.
    """
    rotation = quat_to_rot(flu_enu_quaternion(q_frd_ned_wxyz))
    pitch = math.asin(max(-1.0, min(1.0, float(-rotation[2, 0]))))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    w, x, y, z = quat_from_rpy(roll, pitch, 0.0)
    return (float(x), float(y), float(z), float(w))


def yaw_from_quat(q_wxyz) -> float:
    """Yaw about Z, in radians, for a Z-up quaternion."""
    w, x, y, z = quat_normalize(np.asarray(q_wxyz, dtype=float))
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
