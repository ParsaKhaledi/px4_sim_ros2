"""Camera mount, sensor origins, and the optical-frame rotation.

The mount is FLU metres on the drone body. Positive pitch points the glass
down. Image headers use the optical child of each sensor frame.
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from env import env_float
from hardware import BASELINE_M, DEFAULT_CAM_X, DEFAULT_CAM_Y, DEFAULT_CAM_Z, DEFAULT_PITCH_DEG, FRONT_X_M

OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

LINK_NAME = "camera_link"
IMU_FRAME = "imu_link"
BASE_FRAME = "base_link"

STEREO_LEFT_OPTICAL = "stereo_left_camera_optical_frame"
STEREO_RIGHT_OPTICAL = "stereo_right_camera_optical_frame"
RGB_OPTICAL = "camera_rgb_optical_frame"

RIGHT_INFO_IN = "/camera/stereo/right/camera_info"
RIGHT_INFO_OUT = "/camera/stereo/right/camera_info_baseline"
IMU_TOPIC = "/imu"

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


@dataclass(slots=True, frozen=True)
class Mount:
    """Pose of the camera link on the drone body, in FLU metres and degrees.

    Positive pitch points the glass down. In both SDF and URDF, rpy is
    Rz(yaw) Ry(pitch) Rx(roll), and Ry(+pitch) rotates +x toward -z.
    """

    x: float = DEFAULT_CAM_X
    y: float = DEFAULT_CAM_Y
    z: float = DEFAULT_CAM_Z
    pitch_deg: float = DEFAULT_PITCH_DEG

    @property
    def pitch_rad(self) -> float:
        """Downward pitch in radians."""
        return math.radians(self.pitch_deg)

    def pose_text(self) -> str:
        """SDF pose of this mount, pitch about y."""
        return pose_text(self.x, self.y, self.z, 0.0, self.pitch_rad, 0.0)


def mount_from_env(env: dict[str, str | None] | None = None) -> Mount:
    """Mount from ``CAM_X``, ``CAM_Y``, ``CAM_Z``, and ``CAM_PITCH_DEG``."""
    source: Mapping[str, str | None] = os.environ if env is None else env
    return Mount(
        x=env_float(source, "CAM_X", DEFAULT_CAM_X),
        y=env_float(source, "CAM_Y", DEFAULT_CAM_Y),
        z=env_float(source, "CAM_Z", DEFAULT_CAM_Z),
        pitch_deg=env_float(source, "CAM_PITCH_DEG", DEFAULT_PITCH_DEG),
    )


def pose_text(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> str:
    """Six-number SDF pose, metres and radians, six digits."""
    return f"{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}"


def rot_y(pitch_rad: float) -> Mat3:
    """Right-handed rotation about y, as a tuple of rows."""
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def matvec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Vec3:
    """Multiply a 3x3 matrix by a 3-vector."""
    return tuple(
        matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2]
        for row in range(3)
    )


def matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> Mat3:
    """Multiply two 3x3 matrices."""
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def rpy_matrix(roll: float, pitch: float, yaw: float) -> Mat3:
    """URDF/SDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return matmul(rz, matmul(ry, rx))


def optical_rotation() -> Mat3:
    """Camera-link to optical frame: z forward, x right, y down.

    That is ``rpy = (-pi/2, 0, -pi/2)``. Gazebo looks along +x; image
    headers use this optical child.
    """
    return rpy_matrix(*OPTICAL_RPY)


def _add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    """Add two 3-vectors."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sensor_layouts() -> dict[str, Vec3]:
    """Sensor origins in the camera link, before the mount pitch.

    Left is +y (FLU left). The color camera and the aligned depth sensor
    share the center of the front glass.
    """
    return {
        "stereo_left": (FRONT_X_M, BASELINE_M / 2.0, 0.0),
        "stereo_right": (FRONT_X_M, -BASELINE_M / 2.0, 0.0),
        "rgb": (FRONT_X_M, 0.0, 0.0),
        "imu": (0.0, 0.0, 0.0),
    }


def optical_origin_in_base(sensor: str, mount: Mount | None = None) -> Vec3:
    """Origin of a sensor optical frame expressed in base_link.

    The optical joint has zero translation, so this is also the camera-frame
    origin. The mount pitch rotates the housing, then the offset is added.
    """
    mount = mount or Mount()
    offset = sensor_layouts()[sensor]
    rotated = matvec(rot_y(mount.pitch_rad), offset)
    return _add((mount.x, mount.y, mount.z), rotated)


def right_in_left_optical() -> Vec3:
    """Where the right optical frame sits in the left optical frame.

    Expect about (+baseline, 0, 0): image-right, same distance as the hardware.
    """
    left = sensor_layouts()["stereo_left"]
    right = sensor_layouts()["stereo_right"]
    delta = (right[0] - left[0], right[1] - left[1], right[2] - left[2])
    rotation = optical_rotation()
    transposed = tuple(tuple(rotation[row][col] for row in range(3)) for col in range(3))
    return matvec(transposed, delta)


def optical_rotation_in_base(mount: Mount | None = None) -> Mat3:
    """Orientation of an optical frame in base_link. Same for every sensor,
    because they share the mount pitch and the optical joint has no extra yaw.
    """
    mount = mount or Mount()
    return matmul(rot_y(mount.pitch_rad), optical_rotation())


def rotation_to_quaternion(matrix: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a rotation matrix."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / s
        x = 0.25 * s
        y = (matrix[0][1] + matrix[1][0]) / s
        z = (matrix[0][2] + matrix[2][0]) / s
    elif matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / s
        x = (matrix[0][1] + matrix[1][0]) / s
        y = 0.25 * s
        z = (matrix[1][2] + matrix[2][1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / s
        x = (matrix[0][2] + matrix[2][0]) / s
        y = (matrix[1][2] + matrix[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def optical_quaternion_in_base(
    mount: Mount | None = None,
) -> tuple[float, float, float, float]:
    """Optical-frame orientation in ``base_link``, as ``(x, y, z, w)``."""
    return rotation_to_quaternion(optical_rotation_in_base(mount))
