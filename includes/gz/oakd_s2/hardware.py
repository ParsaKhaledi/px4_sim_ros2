"""OAK-D S2 datasheet numbers, calibration, and the housing box.

Sizes, intrinsics, clip planes, and noise densities live here. Profiles
and the renderers read them so those descriptions cannot drift.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# Housing from the OAK-D S2 datasheet: 97 x 29.5 x 22.9 mm, 91 g.
# https://docs.luxonis.com/hardware/products/OAK-D%20S2
# Shop page and the older hardware manual both list the mass as 91 g.
HOUSING_DEPTH_M = 0.0229
HOUSING_WIDTH_M = 0.097
HOUSING_HEIGHT_M = 0.0295
MASS_KG = 0.091
# Dark box tint so the housing is not Gazebo's default white. Not a measured color.
HOUSING_AMBIENT_RGBA = (0.12, 0.12, 0.13, 1.0)
HOUSING_DIFFUSE_RGBA = (0.2, 0.2, 0.22, 1.0)

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

# Stereo clip (m): near stays off the housing; far covers a typical indoor scene.
STEREO_CLIP_NEAR_M = 0.2
STEREO_CLIP_FAR_M = 30.0
# Color clip (m): nearer than stereo so the bracket and close props stay in frame.
COLOR_CLIP_NEAR_M = 0.08
COLOR_CLIP_FAR_M = 50.0
# Depth clip (m): MinZ about 0.2 m at 400p with extended disparity; far is the ideal range (docs/frames.md).
DEPTH_CLIP_NEAR_M = 0.2
DEPTH_CLIP_FAR_M = 12.0

# BNO086 is a fused part and does not publish a raw Gaussian. The densities
# below are the BMI270-class figures from earlier OAK boards (gyro
# 0.008 deg/s/sqrt(Hz), accel 160 ug/sqrt(Hz)), sampled at IMU_HZ with a
# noise bandwidth of rate/2. They are a stand-in so the sim is not noiseless.
GYRO_DENSITY_DPS_SQRT_HZ = 0.008
ACCEL_DENSITY_G_SQRT_HZ = 160e-6
GRAVITY_M_S2 = 9.80665
# Bias stddev stand-in. The BNO086 publishes no raw bias figure; these keep the sim from being bias-free.
GYRO_BIAS_STDDEV_RAD_S = 1.0e-4
ACCEL_BIAS_STDDEV_M_S2 = 1.0e-3

# RTAB-Map's default Vis/MinInliers. Below this the odometry is treated as lost.
TRACKING_MIN_INLIERS = 20


def gyro_stddev_rad_s(imu_hz: float | None = None) -> float:
    """Gyro noise stddev in rad/s. ``None`` uses ``IMU_HZ``."""
    bandwidth_hz = (IMU_HZ if imu_hz is None else float(imu_hz)) / 2.0
    return math.radians(GYRO_DENSITY_DPS_SQRT_HZ * math.sqrt(bandwidth_hz))


def accel_stddev_m_s2(imu_hz: float | None = None) -> float:
    """Accel noise stddev in m/s^2. ``None`` uses ``IMU_HZ``."""
    bandwidth_hz = (IMU_HZ if imu_hz is None else float(imu_hz)) / 2.0
    return ACCEL_DENSITY_G_SQRT_HZ * GRAVITY_M_S2 * math.sqrt(bandwidth_hz)


def box_inertia(
    mass: float, size_x: float, size_y: float, size_z: float
) -> tuple[float, float, float]:
    """Diagonal inertia of a solid box about its center."""
    factor = mass / 12.0
    ixx = factor * (size_y * size_y + size_z * size_z)
    iyy = factor * (size_x * size_x + size_z * size_z)
    izz = factor * (size_x * size_x + size_y * size_y)
    return ixx, iyy, izz


@dataclass(frozen=True)
class HousingSpec:
    """Datasheet mass, solid-box inertia, and housing size in metres."""

    mass_kg: float
    ixx: float
    iyy: float
    izz: float
    depth_m: float
    width_m: float
    height_m: float


def housing_spec() -> HousingSpec:
    """Mass, solid-box inertia, and size from the datasheet constants.

    SDF and URDF both format this one box, so the inertia and the visual
    size cannot drift apart.
    """
    ixx, iyy, izz = box_inertia(MASS_KG, HOUSING_DEPTH_M, HOUSING_WIDTH_M, HOUSING_HEIGHT_M)
    return HousingSpec(
        mass_kg=MASS_KG,
        ixx=ixx,
        iyy=iyy,
        izz=izz,
        depth_m=HOUSING_DEPTH_M,
        width_m=HOUSING_WIDTH_M,
        height_m=HOUSING_HEIGHT_M,
    )
