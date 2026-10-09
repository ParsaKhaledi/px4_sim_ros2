"""ENU/FLU <-> NED/FRD conversions.

ROS REP-103 uses East-North-Up and body Forward-Left-Up.
PX4 uses North-East-Down and body Forward-Right-Down.

Covariance is rotated with ``R @ Σ @ R.T``. Multiplying a variance vector by
``diag(1, -1, -1)`` flips the sign of the y and z terms and is not used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Maps an ENU vector to NED: [N, E, D] = R @ [E, N, U].
R_NED_FROM_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)
# Same matrix; it is symmetric.
R_ENU_FROM_NED = R_NED_FROM_ENU.copy()

# Maps a FLU vector to FRD. Signs square away inside R @ Σ @ R.T.
R_FRD_FROM_FLU = np.diag([1.0, -1.0, -1.0])
R_FLU_FROM_FRD = R_FRD_FROM_FLU.copy()


def enu_to_ned(position_enu: np.ndarray) -> np.ndarray:
    """Convert an ENU position or vector to NED."""
    east, north, up = np.asarray(position_enu, dtype=float).reshape(3)
    return np.array([north, east, -up], dtype=float)


def ned_to_enu(position_ned: np.ndarray) -> np.ndarray:
    """Convert an NED position or vector to ENU."""
    north, east, down = np.asarray(position_ned, dtype=float).reshape(3)
    return np.array([east, north, -down], dtype=float)


def flu_to_frd(vector_flu: np.ndarray) -> np.ndarray:
    """Convert a body-FLU vector (velocity or angular rate) to FRD."""
    x, y, z = np.asarray(vector_flu, dtype=float).reshape(3)
    return np.array([x, -y, -z], dtype=float)


def frd_to_flu(vector_frd: np.ndarray) -> np.ndarray:
    """Convert a body-FRD vector to FLU."""
    x, y, z = np.asarray(vector_frd, dtype=float).reshape(3)
    return np.array([x, -y, -z], dtype=float)


def rotate_covariance(covariance: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Rotate a 3x3 covariance: ``R @ Σ @ R.T``.

    The result stays positive semi-definite when ``covariance`` is.
    """
    sigma = np.asarray(covariance, dtype=float).reshape(3, 3)
    rot = np.asarray(rotation, dtype=float).reshape(3, 3)
    return rot @ sigma @ rot.T


def covariance_block(flat_6x6: np.ndarray, row: int) -> np.ndarray:
    """Return a 3x3 block from a row-major 6x6 covariance."""
    matrix = np.asarray(flat_6x6, dtype=float).reshape(6, 6)
    return matrix[row : row + 3, row : row + 3].copy()


def diagonal_variances(covariance: np.ndarray) -> np.ndarray:
    """Marginal variances. Tiny negative values from roundoff are clipped to 0."""
    diag = np.diag(np.asarray(covariance, dtype=float).reshape(3, 3)).astype(float)
    return np.maximum(diag, 0.0)


def quat_xyzw_to_rot(quat_xyzw: np.ndarray) -> np.ndarray:
    """Rotation matrix from a ROS quaternion ``(x, y, z, w)``."""
    x, y, z, w = np.asarray(quat_xyzw, dtype=float).reshape(4)
    return _quat_to_rot(w, x, y, z)


def quat_wxyz_to_rot(quat_wxyz: np.ndarray) -> np.ndarray:
    """Rotation matrix from a PX4 quaternion ``(w, x, y, z)``."""
    w, x, y, z = np.asarray(quat_wxyz, dtype=float).reshape(4)
    return _quat_to_rot(w, x, y, z)


def _quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    norm = w * w + x * x + y * y + z * z
    if norm < 1e-12:
        return np.eye(3)
    s = 2.0 / norm
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def rot_to_quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    """Hamiltonian ``(w, x, y, z)`` from a rotation matrix."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([w, x, y, z], dtype=float)
    length = np.linalg.norm(quat)
    if length < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if quat[0] < 0.0:
        quat = -quat
    return quat / length


def rot_to_quat_xyzw(rotation: np.ndarray) -> np.ndarray:
    """ROS quaternion ``(x, y, z, w)`` from a rotation matrix."""
    w, x, y, z = rot_to_quat_wxyz(rotation)
    return np.array([x, y, z, w], dtype=float)


def enu_flu_to_ned_frd_rotation(quat_xyzw: np.ndarray) -> np.ndarray:
    """Body-FRD attitude in NED from a body-FLU attitude in ENU."""
    rot_enu_flu = quat_xyzw_to_rot(quat_xyzw)
    return R_NED_FROM_ENU @ rot_enu_flu @ R_FRD_FROM_FLU


def ned_frd_to_enu_flu_rotation(quat_wxyz: np.ndarray) -> np.ndarray:
    """Body-FLU attitude in ENU from a PX4 body-FRD attitude in NED."""
    rot_ned_frd = quat_wxyz_to_rot(quat_wxyz)
    return R_ENU_FROM_NED @ rot_ned_frd @ R_FLU_FROM_FRD


def yaw_ned_from_rotation(rot_ned_frd: np.ndarray) -> float:
    """Yaw of the body forward axis, 0 at North, positive toward East."""
    forward = np.asarray(rot_ned_frd, dtype=float)[:, 0]
    return math.atan2(float(forward[1]), float(forward[0]))


def yaw_ned_from_enu_quat(quat_xyzw: np.ndarray) -> float:
    """PX4 yaw from a ROS orientation quaternion."""
    return yaw_ned_from_rotation(enu_flu_to_ned_frd_rotation(quat_xyzw))


def enu_yaw_to_ned(yaw_enu: float) -> float:
    """Absolute ENU heading to NED yaw.

    ENU yaw 0 faces East, which is NED yaw +90 degrees. Facing north in ENU
    is yaw +90 degrees and identity heading in NED.
    """
    return wrap_pi(math.pi / 2.0 - float(yaw_enu))


def world_local_rotation(local_heading_ned: float, world_yaw_enu: float) -> float:
    """Yaw of the local ENU frame in world ENU.

    ``local_heading_ned`` is ``vehicle_local_position.heading``.
    ``world_yaw_enu`` is the ``world`` -> ``spawn`` yaw. The subtraction is
    in NED, after :func:`enu_yaw_to_ned`, and that difference is also the
    ENU yaw of the local frame relative to the world::

        rotation = wrap(local_heading - enu_yaw_to_ned(world_yaw_enu))

    GPS and mag keep local north on true north, so the vehicle's local
    heading matches its world heading and the rotation is about 0. Vision
    heading is 0 at the spawn, so the rotation is the negated world heading.
    The two results come from the same subtraction. There is no mode switch.
    """
    world_heading_ned = enu_yaw_to_ned(world_yaw_enu)
    return wrap_pi(float(local_heading_ned) - world_heading_ned)


def world_enu_point_to_local(
    x: float,
    y: float,
    spawn_x: float,
    spawn_y: float,
    rotation: float,
    local_east: float,
    local_north: float,
) -> tuple[float, float]:
    """World ENU point into local ENU.

    ``rotation`` is :func:`world_local_rotation`. The offset from the spawn
    is rotated by ``-rotation``, then shifted by the vehicle's local ENU
    position. Local ENU east/north are the NED east/north fields
    (``vehicle_local_position.y`` / ``.x``).
    """
    dx = float(x) - float(spawn_x)
    dy = float(y) - float(spawn_y)
    cos_r = math.cos(float(rotation))
    sin_r = math.sin(float(rotation))
    east = cos_r * dx + sin_r * dy + float(local_east)
    north = -sin_r * dx + cos_r * dy + float(local_north)
    return east, north


def wrap_pi(angle: float) -> float:
    """Wrap an angle in radians to ``(-pi, pi]``."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped <= -math.pi:
        return math.pi
    return wrapped


def body_horizontal_to_ned(
    vx: float,
    vy: float,
    yaw_rate: float,
    yaw_ned: float,
) -> tuple[float, float, float]:
    """Rotate a level body velocity into NED using yaw only.

    ``vx`` is forward and ``vy`` is left, both in the ROS body frame.
    ``yaw_rate`` is the ROS yaw rate (positive up). The mapping is::

        vy' = -vy
        vN = vx cos(psi) - vy' sin(psi)
        vE = vx sin(psi) + vy' cos(psi)
        yawspeed = -yaw_rate

    Roll and pitch are intentionally not part of this rotation. Nav2
    ``cmd_vel`` is a planar command.
    """
    vy_right = -vy
    cos_psi = math.cos(yaw_ned)
    sin_psi = math.sin(yaw_ned)
    v_north = vx * cos_psi - vy_right * sin_psi
    v_east = vx * sin_psi + vy_right * cos_psi
    return v_north, v_east, -yaw_rate


def body_offset_to_enu(
    forward: float,
    left: float,
    up: float,
    yaw_ned: float,
) -> np.ndarray:
    """Level body displacement (forward, left, up) to an ENU offset.

    Uses the same yaw-only rotation as :func:`body_horizontal_to_ned`.
    """
    v_north, v_east, _unused = body_horizontal_to_ned(forward, left, 0.0, yaw_ned)
    return np.array([v_east, v_north, up], dtype=float)


def command_age(command_time_s: float, now_s: float) -> float:
    """Seconds since the last command.

    Both arguments are already in seconds. The previous offboard script
    divided ``(seconds + nanoseconds)`` by ``1e9`` in the hold branch, which
    made the timeout comparison meaningless.
    """
    return now_s - command_time_s


@dataclass(frozen=True)
class NedAttitude:
    """PX4 orientation fields derived from a ROS pose."""

    quaternion_wxyz: np.ndarray
    yaw: float
    rotation: np.ndarray


def attitude_from_enu_quat(quat_xyzw: np.ndarray) -> NedAttitude | None:
    """Build a NED/FRD attitude, or ``None`` when the quaternion is unusable."""
    quat = np.asarray(quat_xyzw, dtype=float).reshape(4)
    if not np.all(np.isfinite(quat)) or float(np.linalg.norm(quat)) < 1e-6:
        return None
    rotation = enu_flu_to_ned_frd_rotation(quat)
    return NedAttitude(
        quaternion_wxyz=rot_to_quat_wxyz(rotation),
        yaw=yaw_ned_from_rotation(rotation),
        rotation=rotation,
    )
