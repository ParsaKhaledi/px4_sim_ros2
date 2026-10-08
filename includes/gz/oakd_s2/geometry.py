"""OAK-D S2 geometry shared by Gazebo, the URDF, and the stereo baseline fix.

The numbers here are the only place camera size, intrinsics, and the stereo
baseline are defined. ``render_oakd.py`` turns them into SDF and URDF so the
two descriptions cannot drift apart.

Frame convention inside the camera link (REP-103, same as Gazebo):
x forward out of the glass, y left, z up. Gazebo cameras look along +x.
Image headers use the optical child frame (z forward, x right, y down),
reached by rpy = (-pi/2, 0, -pi/2).

The implementation lives in ``hardware``, ``profiles``, ``gpu``,
``intrinsics``, and ``frames``. This module re-exports those names so
``import geometry as geo`` keeps working.
"""

from __future__ import annotations

from env import env_float, env_float as _env_float, env_int, env_int as _env_int, env_str
from frames import (
    BASE_FRAME,
    IMU_FRAME,
    IMU_TOPIC,
    LINK_NAME,
    OPTICAL_RPY,
    RGB_OPTICAL,
    RIGHT_INFO_IN,
    RIGHT_INFO_OUT,
    STEREO_LEFT_OPTICAL,
    STEREO_RIGHT_OPTICAL,
    Mount,
    _add,
    matmul,
    matvec,
    mount_from_env,
    optical_origin_in_base,
    optical_quaternion_in_base,
    optical_rotation,
    optical_rotation_in_base,
    pose_text,
    right_in_left_optical,
    rot_y,
    rotation_to_quaternion,
    rpy_matrix,
    sensor_layouts,
)
from gpu import _nvidia_smi_ok, gpu_visible
from hardware import (
    ACCEL_BIAS_STDDEV_M_S2,
    ACCEL_DENSITY_G_SQRT_HZ,
    ALLOWED_STEREO_RESOLUTIONS,
    BASELINE_M,
    CAMERA_HZ,
    COLOR_CLIP_FAR_M,
    COLOR_CLIP_NEAR_M,
    COLOR_HEIGHT,
    COLOR_HFOV_DEG,
    COLOR_VFOV_DEG,
    COLOR_WIDTH,
    CPU_CAMERA_HZ,
    CPU_COLOR_HEIGHT,
    CPU_COLOR_WIDTH,
    CPU_IMU_HZ,
    CPU_STEREO_HEIGHT,
    CPU_STEREO_WIDTH,
    DEFAULT_CAM_X,
    DEFAULT_CAM_Y,
    DEFAULT_CAM_Z,
    DEFAULT_PITCH_DEG,
    DEPTH_CLIP_FAR_M,
    DEPTH_CLIP_NEAR_M,
    FRONT_X_M,
    GRAVITY_M_S2,
    GYRO_BIAS_STDDEV_RAD_S,
    GYRO_DENSITY_DPS_SQRT_HZ,
    HD_MIN_HEIGHT,
    HD_MIN_WIDTH,
    HOUSING_AMBIENT_RGBA,
    HOUSING_DEPTH_M,
    HOUSING_DIFFUSE_RGBA,
    HOUSING_HEIGHT_M,
    HOUSING_WIDTH_M,
    HW_CAMERA_HZ,
    HW_IMU_HZ,
    IMAGE_NOISE_STDDEV,
    IMU_HZ,
    LEFT_INTRINSICS,
    MASS_KG,
    NATIVE_STEREO_HEIGHT,
    NATIVE_STEREO_WIDTH,
    RIGHT_INTRINSICS,
    STEREO_CLIP_FAR_M,
    STEREO_CLIP_NEAR_M,
    TRACKING_MIN_INLIERS,
    accel_stddev_m_s2,
    box_inertia,
    gyro_stddev_rad_s,
    HousingSpec,
    housing_spec,
)
from intrinsics import (
    _color_full,
    color_intrinsics,
    corrected_projection,
    diagonal_fov_deg,
    hfov_from_intrinsics,
    rectified_k,
    rectified_p,
    scale_intrinsics,
    stereo_intrinsics,
    stereo_tx,
    vfov_from_intrinsics,
)
from profiles import (
    CPU_PROFILE,
    FULL_PROFILE,
    HW_PROFILE,
    PROFILES,
    VisionProfile,
    _allowed_stereo_text,
    _hd_floor_text,
    _parse_stereo_res,
    _real_use_stereo_text,
    _require_allowed_stereo,
    _resolution_text,
    _stereo_size,
    no_gpu_warning,
    profile_from_env,
    rate_text,
    sub_hd_warning,
)
