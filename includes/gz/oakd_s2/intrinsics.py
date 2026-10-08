"""Pinhole intrinsics, rectified K and P, and field of view.

Stereo K is the left calibration scaled to the profile size. Both cameras
share it. Right ``P[3]`` is ``Tx = -fx * baseline``.
"""

from __future__ import annotations

import math

from hardware import BASELINE_M, COLOR_HEIGHT, COLOR_HFOV_DEG, COLOR_WIDTH, LEFT_INTRINSICS
from profiles import FULL_PROFILE, VisionProfile

Intrinsics = dict[str, float | int]


def scale_intrinsics(src: Intrinsics, width: int, height: int) -> Intrinsics:
    """Scale a pinhole to a new resolution. Horizontal and vertical FOV stay."""
    if width <= 0 or height <= 0:
        raise ValueError(f"resolution must be positive, got {width}x{height}")
    sx = float(width) / float(src["width"])
    sy = float(height) / float(src["height"])
    return {
        "width": int(width),
        "height": int(height),
        "fx": float(src["fx"]) * sx,
        "fy": float(src["fy"]) * sy,
        "cx": float(src["cx"]) * sx,
        "cy": float(src["cy"]) * sy,
    }


def stereo_intrinsics(profile: VisionProfile | None = None) -> Intrinsics:
    """Left calibration at this profile's stereo resolution. Both cameras share it."""
    chosen = FULL_PROFILE if profile is None else profile
    return scale_intrinsics(LEFT_INTRINSICS, chosen.stereo_width, chosen.stereo_height)


def rectified_k(profile: VisionProfile | None = None) -> list[float]:
    """3x3 K shared by both stereo cameras. Row-major, from the left calibration."""
    src = stereo_intrinsics(profile)
    return [src["fx"], 0.0, src["cx"], 0.0, src["fy"], src["cy"], 0.0, 0.0, 1.0]


def rectified_p(
    right: bool,
    baseline_m: float = BASELINE_M,
    profile: VisionProfile | None = None,
) -> list[float]:
    """3x4 ROS ``camera_info`` P, row-major. Right ``P[3]`` is Tx."""
    src = stereo_intrinsics(profile)
    tx = stereo_tx(src["fx"], baseline_m) if right else 0.0
    return [
        src["fx"], 0.0, src["cx"], tx,
        0.0, src["fy"], src["cy"], 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]


def stereo_tx(fx: float, baseline_m: float = BASELINE_M) -> float:
    """OpenCV stereo Tx. Right camera sits at +x in the left optical frame,
    and the projection matrix stores that as Tx = -fx * baseline.
    """
    if fx <= 0.0:
        raise ValueError(f"fx must be positive, got {fx}")
    if baseline_m <= 0.0:
        raise ValueError(f"baseline must be positive, got {baseline_m}")
    return -fx * baseline_m


def corrected_projection(
    p: list[float] | tuple[float, ...],
    k: list[float] | tuple[float, ...],
    baseline_m: float = BASELINE_M,
) -> list[float] | None:
    """Return a 12-element P with Tx set from this camera's fx.

    Two separate Gazebo cameras each publish Tx = 0. Harmonic does copy
    ``<projection><tx>`` into P[3] (gz-sensors CameraSensor::PopulateInfo),
    but a relay still overwrites P[3] so a build that leaves it at 0 is fixed.
    Returns None when fx cannot be recovered.
    """
    values = [float(v) for v in p]
    if len(values) != 12:
        raise ValueError("camera_info P has 12 elements")
    k_values = [float(v) for v in k]
    fx = k_values[0] if k_values[0] else values[0]
    fy = k_values[4] if len(k_values) > 4 and k_values[4] else values[5]
    cx = k_values[2] if len(k_values) > 2 and k_values[2] else values[2]
    cy = k_values[5] if len(k_values) > 5 and k_values[5] else values[6]
    if fx <= 0.0 or fy <= 0.0:
        return None
    if values[0] == 0.0:
        values[0] = fx
        values[2] = cx
        values[5] = fy
        values[6] = cy
        values[10] = 1.0
    values[3] = stereo_tx(fx, baseline_m)
    return values


def hfov_from_intrinsics(intrinsics: Intrinsics) -> float:
    """Horizontal field of view in radians that matches fx."""
    half_width = intrinsics["width"] / 2.0
    return 2.0 * math.atan(half_width / intrinsics["fx"])


def vfov_from_intrinsics(intrinsics: Intrinsics) -> float:
    """Vertical field of view in radians that matches fy."""
    half_height = intrinsics["height"] / 2.0
    return 2.0 * math.atan(half_height / intrinsics["fy"])


def _color_full() -> Intrinsics:
    """Square-pixel pinhole from the AF datasheet HFOV at 640x480.

    The datasheet also lists VFOV 54 deg and DFOV 78 deg. Those three angles
    are not one pinhole on a 4:3 sensor (66 by 54 would be about 79 deg on
    the diagonal). HFOV is the number this sim is pinned to, and square
    pixels then imply about 52 deg vertical and 78 deg diagonal.
    """
    hfov = math.radians(COLOR_HFOV_DEG)
    fx = (COLOR_WIDTH / 2.0) / math.tan(hfov / 2.0)
    return {
        "width": COLOR_WIDTH,
        "height": COLOR_HEIGHT,
        "fx": fx,
        "fy": fx,
        "cx": COLOR_WIDTH / 2.0,
        "cy": COLOR_HEIGHT / 2.0,
    }


def color_intrinsics(profile: VisionProfile | None = None) -> Intrinsics:
    """Color pinhole at the profile resolution. FOV matches the 640x480 spec."""
    chosen = FULL_PROFILE if profile is None else profile
    return scale_intrinsics(_color_full(), chosen.color_width, chosen.color_height)


def diagonal_fov_deg(hfov_deg: float, vfov_deg: float) -> float:
    """Diagonal field of view for a rectilinear lens, in degrees."""
    th = math.tan(math.radians(hfov_deg) / 2.0)
    tv = math.tan(math.radians(vfov_deg) / 2.0)
    return math.degrees(2.0 * math.atan(math.sqrt(th * th + tv * tv)))
