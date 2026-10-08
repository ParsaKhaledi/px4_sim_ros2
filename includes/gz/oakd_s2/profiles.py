"""Vision profiles, environment overrides, and the startup warnings.

``cpu``, ``full``, and ``hw`` are defined here. ``CAM_*`` overrides are
applied on top. The GPU probe itself lives in ``gpu``.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass

from env import env_float, env_int, env_str
from gpu import gpu_visible
from hardware import (
    ALLOWED_STEREO_RESOLUTIONS,
    CAMERA_HZ,
    COLOR_HEIGHT,
    COLOR_WIDTH,
    CPU_CAMERA_HZ,
    CPU_COLOR_HEIGHT,
    CPU_COLOR_WIDTH,
    CPU_IMU_HZ,
    CPU_STEREO_HEIGHT,
    CPU_STEREO_WIDTH,
    HD_MIN_HEIGHT,
    HD_MIN_WIDTH,
    HW_CAMERA_HZ,
    HW_IMU_HZ,
    IMU_HZ,
    LEFT_INTRINSICS,
    NATIVE_STEREO_HEIGHT,
    NATIVE_STEREO_WIDTH,
)


@dataclass(frozen=True)
class VisionProfile:
    """Image size and sensor rate for one ``VISION_PROFILE``.

    ``cpu`` is the default: 320x200 for CPU-only simulation and CI.
    ``full`` is native 1280x800 stereo for a GPU simulation. ``hw`` is
    1280x800 at a lower rate for a real OAK-D S2 on a small onboard computer.
    ``full`` and ``hw`` are opt-in.

    Optional ``CAM_STEREO_RES`` (or ``CAM_STEREO_WIDTH`` and
    ``CAM_STEREO_HEIGHT``), ``CAM_RATE_HZ``, and the other ``CAM_*`` /
    ``IMU_RATE_HZ`` overrides replace individual fields after the profile is
    chosen. Stereo intrinsics are scaled from the 640x400 calibration so the
    field of view does not change. Only the 16:10 sizes in
    ``ALLOWED_STEREO_RESOLUTIONS`` are accepted.
    """

    name: str
    stereo_width: int
    stereo_height: int
    color_width: int
    color_height: int
    camera_hz: float
    imu_hz: float


FULL_PROFILE = VisionProfile(
    "full",
    NATIVE_STEREO_WIDTH,
    NATIVE_STEREO_HEIGHT,
    COLOR_WIDTH,
    COLOR_HEIGHT,
    CAMERA_HZ,
    IMU_HZ,
)
CPU_PROFILE = VisionProfile(
    "cpu",
    CPU_STEREO_WIDTH,
    CPU_STEREO_HEIGHT,
    CPU_COLOR_WIDTH,
    CPU_COLOR_HEIGHT,
    CPU_CAMERA_HZ,
    CPU_IMU_HZ,
)
HW_PROFILE = VisionProfile(
    "hw",
    NATIVE_STEREO_WIDTH,
    NATIVE_STEREO_HEIGHT,
    COLOR_WIDTH,
    COLOR_HEIGHT,
    HW_CAMERA_HZ,
    HW_IMU_HZ,
)
PROFILES = {
    FULL_PROFILE.name: FULL_PROFILE,
    CPU_PROFILE.name: CPU_PROFILE,
    HW_PROFILE.name: HW_PROFILE,
}


def _resolution_text(width: int, height: int) -> str:
    """``WIDTHxHEIGHT`` text for one size."""
    return f"{int(width)}x{int(height)}"


def _hd_floor_text() -> str:
    """HD warning floor, from ``HD_MIN_WIDTH`` and ``HD_MIN_HEIGHT``."""
    return _resolution_text(HD_MIN_WIDTH, HD_MIN_HEIGHT)


def _real_use_stereo_text() -> str:
    """Allowed stereo size that meets the HD floor."""
    for width, height in ALLOWED_STEREO_RESOLUTIONS:
        if width >= HD_MIN_WIDTH and height >= HD_MIN_HEIGHT:
            return _resolution_text(width, height)
    raise RuntimeError("ALLOWED_STEREO_RESOLUTIONS has no size at or above the HD floor")


def _parse_stereo_res(text: str) -> tuple[int, int]:
    """Parse ``1280x800``-style text into a width and height."""
    cleaned = str(text).strip().lower().replace(" ", "")
    if "x" not in cleaned:
        raise ValueError(
            f"CAM_STEREO_RES must look like {_real_use_stereo_text()}, got {text}. "
            f"{_allowed_stereo_text()}"
        )
    width_text, height_text = cleaned.split("x", 1)
    try:
        width = int(float(width_text))
        height = int(float(height_text))
    except ValueError as exc:
        raise ValueError(
            f"CAM_STEREO_RES must look like {_real_use_stereo_text()}, got {text}. {_allowed_stereo_text()}"
        ) from exc
    return width, height


def _allowed_stereo_text() -> str:
    """Allowed sizes, and why a crop such as 1280x720 is rejected."""
    sizes = ", ".join(
        _resolution_text(width, height) for width, height in ALLOWED_STEREO_RESOLUTIONS
    )
    calibration = _resolution_text(LEFT_INTRINSICS["width"], LEFT_INTRINSICS["height"])
    return (
        f"Allowed stereo sizes are {sizes}, the uniform 16:10 scales of the "
        f"{calibration} calibration. {_hd_floor_text()} is rejected because it would crop "
        f"{_real_use_stereo_text()} and move the principal point."
    )


def _require_allowed_stereo(width: int, height: int) -> None:
    """Raise unless this pair is in ``ALLOWED_STEREO_RESOLUTIONS``."""
    if (int(width), int(height)) in ALLOWED_STEREO_RESOLUTIONS:
        return
    raise ValueError(f"stereo resolution {width}x{height} is not valid. {_allowed_stereo_text()}")


def _stereo_size(source: Mapping[str, str | None], base: VisionProfile) -> tuple[int, int]:
    """``CAM_STEREO_RES`` or the width/height pair. They must name one allowed size."""
    res = source.get("CAM_STEREO_RES")
    res_set = env_str(source, "CAM_STEREO_RES") is not None
    width_set = env_str(source, "CAM_STEREO_WIDTH") is not None
    height_set = env_str(source, "CAM_STEREO_HEIGHT") is not None
    if res_set:
        width, height = _parse_stereo_res(str(res))
        if width_set or height_set:
            other_width = env_int(source, "CAM_STEREO_WIDTH", width) if width_set else width
            other_height = env_int(source, "CAM_STEREO_HEIGHT", height) if height_set else height
            if (other_width, other_height) != (width, height):
                raise ValueError(
                    f"CAM_STEREO_RES={str(res).strip()} disagrees with "
                    f"CAM_STEREO_WIDTH/CAM_STEREO_HEIGHT {other_width}x{other_height}"
                )
        _require_allowed_stereo(width, height)
        return width, height
    width = env_int(source, "CAM_STEREO_WIDTH", base.stereo_width)
    height = env_int(source, "CAM_STEREO_HEIGHT", base.stereo_height)
    _require_allowed_stereo(width, height)
    return width, height


def profile_from_env(env: dict[str, str | None] | None = None) -> VisionProfile:
    """``VISION_PROFILE`` selects ``cpu``, ``full``, or ``hw``. Overrides win.

    The default profile is ``cpu``. An empty ``VISION_PROFILE`` is ``cpu``.
    """
    source: Mapping[str, str | None] = os.environ if env is None else env
    raw = source.get("VISION_PROFILE", "cpu")
    selected = env_str(source, "VISION_PROFILE", "cpu")
    name = "cpu" if selected is None else selected.strip().lower()
    if name not in PROFILES:
        raise ValueError(f"VISION_PROFILE must be cpu, full, or hw, got {raw}")
    base = PROFILES[name]
    camera_hz = env_float(source, "CAM_RATE_HZ", base.camera_hz)
    imu_hz = env_float(source, "IMU_RATE_HZ", base.imu_hz)
    if camera_hz <= 0.0 or imu_hz <= 0.0:
        raise ValueError(f"sensor rates must be positive, got camera {camera_hz} imu {imu_hz}")
    stereo_width, stereo_height = _stereo_size(source, base)
    return VisionProfile(
        name=name,
        stereo_width=stereo_width,
        stereo_height=stereo_height,
        color_width=env_int(source, "CAM_COLOR_WIDTH", base.color_width),
        color_height=env_int(source, "CAM_COLOR_HEIGHT", base.color_height),
        camera_hz=camera_hz,
        imu_hz=imu_hz,
    )


def no_gpu_warning(profile: VisionProfile, gpu_present: bool | None = None) -> str | None:
    """One warning when ``full`` or ``hw`` is selected and no GPU is visible.

    This does not reject the profile. ``cpu`` does not warn. Call it from the
    process that renders the Gazebo cameras (the PX4 container writing SDFs).
    The Rtabmap container has no ``/dev`` and would always warn.
    """
    if profile.name not in ("full", "hw"):
        return None
    present = gpu_visible() if gpu_present is None else gpu_present
    if present:
        return None
    return (
        f"warning: VISION_PROFILE={profile.name} expects a GPU or onboard hardware, "
        f"but no GPU is visible (no /dev/dri/renderD* and nvidia-smi is not working). "
        f"Set VISION_PROFILE=cpu on this machine."
    )


def sub_hd_warning(profile: VisionProfile) -> str | None:
    """Warning when ``full`` or ``hw`` is running below HD stereo.

    ``cpu`` is the low-resolution profile and does not warn. 1280x800 is the
    only allowed size that is not below 1280x720.
    """
    if profile.name not in ("full", "hw"):
        return None
    if profile.stereo_width >= HD_MIN_WIDTH and profile.stereo_height >= HD_MIN_HEIGHT:
        return None
    return (
        f"warning: VISION_PROFILE={profile.name} stereo "
        f"{profile.stereo_width}x{profile.stereo_height} is below {_hd_floor_text()}. "
        f"{_real_use_stereo_text()} is the stereo size for real use. The cpu profile is the "
        f"low-resolution option for CPU-only simulation and CI."
    )
