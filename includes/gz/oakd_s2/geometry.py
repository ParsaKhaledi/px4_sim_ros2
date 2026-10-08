"""OAK-D S2 geometry shared by Gazebo, the URDF, and the stereo baseline fix.

The numbers here are the only place camera size, intrinsics, and the stereo
baseline are defined. ``render_oakd.py`` turns them into SDF and URDF so the
two descriptions cannot drift apart.

Frame convention inside the camera link (REP-103, same as Gazebo):
x forward out of the glass, y left, z up. Gazebo cameras look along +x.
Image headers use the optical child frame (z forward, x right, y down),
reached by rpy = (-pi/2, 0, -pi/2).
"""

from __future__ import annotations

import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path


# Housing from the OAK-D S2 datasheet: 97 x 29.5 x 22.9 mm, 91 g.
# https://docs.luxonis.com/hardware/products/OAK-D%20S2
# Shop page and the older hardware manual both list the mass as 91 g.
HOUSING_DEPTH_M = 0.0229
HOUSING_WIDTH_M = 0.097
HOUSING_HEIGHT_M = 0.0295
MASS_KG = 0.091

# Fixed stereo baseline. Luxonis' current product page says "75cm"; that is a
# typo. The shop page, the older manual, and depthai-ros all say 7.5 cm.
BASELINE_M = 0.075

# Lenses sit on the front glass. The link origin is the housing center, which
# is the same idea as depthai_descriptions (cameras share one origin, split
# only by the baseline) plus this glass offset.
FRONT_X_M = HOUSING_DEPTH_M / 2.0

# PX4 x500_depth includes model://OakD-Lite at this pose, with no rotation.
# The link inside that model is camera_link (the fixed joint's child).
DEFAULT_CAM_X = 0.12
DEFAULT_CAM_Y = 0.03
DEFAULT_CAM_Z = 0.242
# 17 deg is the old 0.3 rad downward pitch, now applied to the whole mount.
DEFAULT_PITCH_DEG = 17.0

# Real S2 calibration at 640x400, from
# includes/gazebo_classic/Params/OAK-D Calibration Files/oak-s2/.
# The nominal datasheet HFOV is 80 deg. These intrinsics are narrower
# (about 76.4 deg). The sim follows the left calibration and sets
# horizontal_fov from fx so the rendered image and camera_info describe
# the same pinhole.
LEFT_INTRINSICS = {
    "width": 640,
    "height": 400,
    "fx": 406.6239117362143,
    "fy": 406.080036476775,
    "cx": 305.77686139278046,
    "cy": 204.52975717455422,
}
# The real right camera is a few pixels different. Both renders use the left
# K instead. RTAB-Map treats the pair as already rectified, and two Ks bias
# the depth. Kept here so the difference stays visible.
RIGHT_INTRINSICS = {
    "width": 640,
    "height": 400,
    "fx": 407.2771382926294,
    "fy": 407.13351237406044,
    "cx": 304.9249183482313,
    "cy": 203.90584453159025,
}

# Center color camera, auto-focus variant (IMX378). Datasheet FOV 78/66/54.
# The fixed-focus variant is 82/69/55. AF is the variant in the datasheet's
# "center color camera" table. 640x480 keeps the 4:3 sensor aspect.
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
COLOR_HFOV_DEG = 66.0
# Datasheet vertical FOV. Not used to build the pinhole; see color_intrinsics.
COLOR_VFOV_DEG = 54.0

CAMERA_HZ = 30
IMU_HZ = 200

# Native OV9282 size. The calibration above is the 2x2 bin of this frame.
# VISION_PROFILE=full and hw render this size. cpu does not.
NATIVE_STEREO_WIDTH = 1280
NATIVE_STEREO_HEIGHT = 800

# VISION_PROFILE=cpu. Half the calibration pixels on each axis (640x400 ->
# 320x200), half the color frame (640x480 -> 320x240), and slower sensors.
# This is the low-resource set for CPU-only simulation and CI, not a mode for
# the real camera. Field of view stays put because fx, fy, cx, and cy scale
# with the resolution.
CPU_STEREO_WIDTH = 320
CPU_STEREO_HEIGHT = 200
CPU_COLOR_WIDTH = 320
CPU_COLOR_HEIGHT = 240
CPU_CAMERA_HZ = 10
CPU_IMU_HZ = 100

# VISION_PROFILE=hw. Same native stereo size as full, at a rate a modest
# onboard computer can track. Color stays at the 640x480 datasheet frame.
HW_CAMERA_HZ = 15
HW_IMU_HZ = 200

# Uniform scales of the 640x400 calibration (16:10). sx and sy match, so the
# principal point stays the scaled calibration point. 1280x720 is not here:
# it would crop 1280x800 and move cx/cy.
ALLOWED_STEREO_RESOLUTIONS = (
    (NATIVE_STEREO_WIDTH, NATIVE_STEREO_HEIGHT),
    (LEFT_INTRINSICS["width"], LEFT_INTRINSICS["height"]),
    (CPU_STEREO_WIDTH, CPU_STEREO_HEIGHT),
)

# Real-use stereo is at least HD. full and hw below this log a warning.
HD_MIN_WIDTH = 1280
HD_MIN_HEIGHT = 720

# Ogre's image noise pass is in normalized intensity. 0.007 is about two
# counts on an 8-bit image, enough to not be a perfect render.
IMAGE_NOISE_STDDEV = 0.007

# BNO086 is a fused part and does not publish a raw Gaussian. The densities
# below are the BMI270-class figures from earlier OAK boards (gyro
# 0.008 deg/s/sqrt(Hz), accel 160 ug/sqrt(Hz)), sampled at IMU_HZ with a
# noise bandwidth of rate/2. They are a stand-in so the sim is not noiseless.
GYRO_DENSITY_DPS_SQRT_HZ = 0.008
ACCEL_DENSITY_G_SQRT_HZ = 160e-6
GRAVITY_M_S2 = 9.80665

# RTAB-Map's default Vis/MinInliers. Below this the odometry is treated as lost.
TRACKING_MIN_INLIERS = 20

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


def gyro_stddev_rad_s(imu_hz: float | None = None) -> float:
    bandwidth_hz = (IMU_HZ if imu_hz is None else float(imu_hz)) / 2.0
    return math.radians(GYRO_DENSITY_DPS_SQRT_HZ * math.sqrt(bandwidth_hz))


def accel_stddev_m_s2(imu_hz: float | None = None) -> float:
    bandwidth_hz = (IMU_HZ if imu_hz is None else float(imu_hz)) / 2.0
    return ACCEL_DENSITY_G_SQRT_HZ * GRAVITY_M_S2 * math.sqrt(bandwidth_hz)


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


def _env_int(env: dict, name: str, default: int) -> int:
    raw = env.get(name)
    if raw is None or str(raw).strip() == "":
        return int(default)
    value = int(float(raw))
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {raw}")
    return value


def _env_float(env: dict, name: str, default: float) -> float:
    raw = env.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return float(raw)


def _parse_stereo_res(text: str) -> tuple[int, int]:
    cleaned = str(text).strip().lower().replace(" ", "")
    if "x" not in cleaned:
        raise ValueError(
            f"CAM_STEREO_RES must look like 1280x800, got {text}. "
            f"{_allowed_stereo_text()}"
        )
    width_text, height_text = cleaned.split("x", 1)
    try:
        width = int(float(width_text))
        height = int(float(height_text))
    except ValueError as exc:
        raise ValueError(
            f"CAM_STEREO_RES must look like 1280x800, got {text}. {_allowed_stereo_text()}"
        ) from exc
    return width, height


def _allowed_stereo_text() -> str:
    sizes = ", ".join(f"{width}x{height}" for width, height in ALLOWED_STEREO_RESOLUTIONS)
    return (
        f"Allowed stereo sizes are {sizes}, the uniform 16:10 scales of the "
        f"640x400 calibration. 1280x720 is rejected because it would crop "
        f"1280x800 and move the principal point."
    )


def _require_allowed_stereo(width: int, height: int) -> None:
    if (int(width), int(height)) in ALLOWED_STEREO_RESOLUTIONS:
        return
    raise ValueError(f"stereo resolution {width}x{height} is not valid. {_allowed_stereo_text()}")


def _stereo_size(source: dict, base: VisionProfile) -> tuple[int, int]:
    """``CAM_STEREO_RES`` or the width/height pair. They must name one allowed size."""
    res = source.get("CAM_STEREO_RES")
    res_set = res is not None and str(res).strip() != ""
    width_raw = source.get("CAM_STEREO_WIDTH")
    height_raw = source.get("CAM_STEREO_HEIGHT")
    width_set = width_raw is not None and str(width_raw).strip() != ""
    height_set = height_raw is not None and str(height_raw).strip() != ""
    if res_set:
        width, height = _parse_stereo_res(str(res))
        if width_set or height_set:
            other_width = _env_int(source, "CAM_STEREO_WIDTH", width) if width_set else width
            other_height = _env_int(source, "CAM_STEREO_HEIGHT", height) if height_set else height
            if (other_width, other_height) != (width, height):
                raise ValueError(
                    f"CAM_STEREO_RES={str(res).strip()} disagrees with "
                    f"CAM_STEREO_WIDTH/CAM_STEREO_HEIGHT {other_width}x{other_height}"
                )
        _require_allowed_stereo(width, height)
        return width, height
    width = _env_int(source, "CAM_STEREO_WIDTH", base.stereo_width)
    height = _env_int(source, "CAM_STEREO_HEIGHT", base.stereo_height)
    _require_allowed_stereo(width, height)
    return width, height


def profile_from_env(env: dict | None = None) -> VisionProfile:
    """``VISION_PROFILE`` selects ``cpu``, ``full``, or ``hw``. Overrides win.

    The default profile is ``cpu``. An empty ``VISION_PROFILE`` is ``cpu``.
    """
    source = os.environ if env is None else env
    raw = source.get("VISION_PROFILE", "cpu")
    name = "cpu" if raw is None or str(raw).strip() == "" else str(raw).strip().lower()
    if name not in PROFILES:
        raise ValueError(f"VISION_PROFILE must be cpu, full, or hw, got {raw}")
    base = PROFILES[name]
    camera_hz = _env_float(source, "CAM_RATE_HZ", base.camera_hz)
    imu_hz = _env_float(source, "IMU_RATE_HZ", base.imu_hz)
    if camera_hz <= 0.0 or imu_hz <= 0.0:
        raise ValueError(f"sensor rates must be positive, got camera {camera_hz} imu {imu_hz}")
    stereo_width, stereo_height = _stereo_size(source, base)
    return VisionProfile(
        name=name,
        stereo_width=stereo_width,
        stereo_height=stereo_height,
        color_width=_env_int(source, "CAM_COLOR_WIDTH", base.color_width),
        color_height=_env_int(source, "CAM_COLOR_HEIGHT", base.color_height),
        camera_hz=camera_hz,
        imu_hz=imu_hz,
    )


def _nvidia_smi_ok() -> bool:
    """True when ``nvidia-smi`` runs and exits 0."""
    try:
        completed = subprocess.run(
            ["nvidia-smi"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


def gpu_visible(dri_nodes: list[str] | None = None, nvidia_ok: bool | None = None) -> bool:
    """A render node under ``/dev/dri`` or a working ``nvidia-smi`` counts."""
    if dri_nodes is None:
        dri = Path("/dev/dri")
        dri_nodes = sorted(str(path) for path in dri.glob("renderD*")) if dri.is_dir() else []
    if nvidia_ok is None:
        nvidia_ok = _nvidia_smi_ok()
    return bool(dri_nodes) or bool(nvidia_ok)


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
        f"{profile.stereo_width}x{profile.stereo_height} is below 1280x720. "
        f"1280x800 is the stereo size for real use. The cpu profile is the "
        f"low-resolution option for CPU-only simulation and CI."
    )


def scale_intrinsics(src: dict, width: int, height: int) -> dict:
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


def stereo_intrinsics(profile: VisionProfile | None = None) -> dict:
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
    """3x4 projection. Same K on both cameras. Only the right camera has Tx."""
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


def corrected_projection(p, k, baseline_m: float = BASELINE_M):
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


def hfov_from_intrinsics(intrinsics: dict) -> float:
    """Horizontal field of view in radians that matches fx."""
    half_width = intrinsics["width"] / 2.0
    return 2.0 * math.atan(half_width / intrinsics["fx"])


def vfov_from_intrinsics(intrinsics: dict) -> float:
    half_height = intrinsics["height"] / 2.0
    return 2.0 * math.atan(half_height / intrinsics["fy"])


def _color_full() -> dict:
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


def color_intrinsics(profile: VisionProfile | None = None) -> dict:
    """Color pinhole at the profile resolution. FOV matches the 640x480 spec."""
    chosen = FULL_PROFILE if profile is None else profile
    return scale_intrinsics(_color_full(), chosen.color_width, chosen.color_height)


def diagonal_fov_deg(hfov_deg: float, vfov_deg: float) -> float:
    """Diagonal field of view for a rectilinear lens, in degrees."""
    th = math.tan(math.radians(hfov_deg) / 2.0)
    tv = math.tan(math.radians(vfov_deg) / 2.0)
    return math.degrees(2.0 * math.atan(math.sqrt(th * th + tv * tv)))


def box_inertia(mass: float, size_x: float, size_y: float, size_z: float):
    """Diagonal inertia of a solid box about its center."""
    factor = mass / 12.0
    ixx = factor * (size_y * size_y + size_z * size_z)
    iyy = factor * (size_x * size_x + size_z * size_z)
    izz = factor * (size_x * size_x + size_y * size_y)
    return ixx, iyy, izz


@dataclass(frozen=True)
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
        return math.radians(self.pitch_deg)

    def pose_text(self) -> str:
        return pose_text(self.x, self.y, self.z, 0.0, self.pitch_rad, 0.0)


def mount_from_env(env: dict | None = None) -> Mount:
    source = os.environ if env is None else env
    return Mount(
        x=_env_float(source, "CAM_X", DEFAULT_CAM_X),
        y=_env_float(source, "CAM_Y", DEFAULT_CAM_Y),
        z=_env_float(source, "CAM_Z", DEFAULT_CAM_Z),
        pitch_deg=_env_float(source, "CAM_PITCH_DEG", DEFAULT_PITCH_DEG),
    )


def pose_text(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}"


def rot_y(pitch_rad: float):
    """Right-handed rotation about y, as a tuple of rows."""
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def matvec(matrix, vector):
    return tuple(
        matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2]
        for row in range(3)
    )


def matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def rpy_matrix(roll: float, pitch: float, yaw: float):
    """URDF/SDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return matmul(rz, matmul(ry, rx))


def optical_rotation():
    return rpy_matrix(*OPTICAL_RPY)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sensor_layouts() -> dict:
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


def optical_origin_in_base(sensor: str, mount: Mount | None = None):
    """Origin of a sensor optical frame expressed in base_link.

    The optical joint has zero translation, so this is also the camera-frame
    origin. The mount pitch rotates the housing, then the offset is added.
    """
    mount = mount or Mount()
    offset = sensor_layouts()[sensor]
    rotated = matvec(rot_y(mount.pitch_rad), offset)
    return _add((mount.x, mount.y, mount.z), rotated)


def right_in_left_optical():
    """Where the right optical frame sits in the left optical frame.

    Expect about (+baseline, 0, 0): image-right, same distance as the hardware.
    """
    left = sensor_layouts()["stereo_left"]
    right = sensor_layouts()["stereo_right"]
    delta = (right[0] - left[0], right[1] - left[1], right[2] - left[2])
    rotation = optical_rotation()
    transposed = tuple(tuple(rotation[row][col] for row in range(3)) for col in range(3))
    return matvec(transposed, delta)


def optical_rotation_in_base(mount: Mount | None = None):
    """Orientation of an optical frame in base_link. Same for every sensor,
    because they share the mount pitch and the optical joint has no extra yaw.
    """
    mount = mount or Mount()
    return matmul(rot_y(mount.pitch_rad), optical_rotation())


def rotation_to_quaternion(matrix):
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


def optical_quaternion_in_base(mount: Mount | None = None):
    return rotation_to_quaternion(optical_rotation_in_base(mount))
