"""Stereo and RGB-D SDF text for the OAK-D S2.

The Gazebo model name stays ``OakD-Lite``. Image size and rate come from
the vision profile. The mount comes from ``frames.Mount``.
"""

from __future__ import annotations

from dataclasses import dataclass

from frames import (
    IMU_FRAME,
    IMU_TOPIC,
    LINK_NAME,
    RGB_OPTICAL,
    RIGHT_INFO_IN,
    STEREO_LEFT_OPTICAL,
    STEREO_RIGHT_OPTICAL,
    Mount,
    pose_text,
    sensor_layouts,
)
from hardware import (
    ACCEL_BIAS_STDDEV_M_S2,
    COLOR_CLIP_FAR_M,
    COLOR_CLIP_NEAR_M,
    DEPTH_CLIP_FAR_M,
    DEPTH_CLIP_NEAR_M,
    GYRO_BIAS_STDDEV_RAD_S,
    HOUSING_AMBIENT_RGBA,
    HOUSING_DIFFUSE_RGBA,
    IMAGE_NOISE_STDDEV,
    STEREO_CLIP_FAR_M,
    STEREO_CLIP_NEAR_M,
    accel_stddev_m_s2,
    gyro_stddev_rad_s,
    housing_spec,
)
from intrinsics import color_intrinsics, hfov_from_intrinsics, stereo_intrinsics, stereo_tx
from profiles import FULL_PROFILE, VisionProfile, rate_text

Intrinsics = dict[str, float | int]
Vec3 = tuple[float, float, float]


def indent(text: str, spaces: int) -> str:
    """Indent every non-empty line by ``spaces`` spaces."""
    pad = " " * spaces
    return "\n".join(pad + line if line else line for line in text.splitlines())


def fmt(value: float) -> str:
    """Stable, full-precision text. Calibration fx must survive a round trip."""
    return f"{value:.16g}"


def _lens(intrinsics: Intrinsics, tx: float) -> str:
    """SDF lens block: calibration intrinsics and projection ``tx``."""
    # scale_to_hfov stays off so fx/fy/cx/cy are the calibration, not a
    # rescaled copy of horizontal_fov. gnomonical is the rectilinear lens;
    # the default stereographic type would be a fisheye.
    return f"""        <lens>
          <type>gnomonical</type>
          <scale_to_hfov>false</scale_to_hfov>
          <intrinsics>
            <fx>{fmt(intrinsics['fx'])}</fx>
            <fy>{fmt(intrinsics['fy'])}</fy>
            <cx>{fmt(intrinsics['cx'])}</cx>
            <cy>{fmt(intrinsics['cy'])}</cy>
            <s>0</s>
          </intrinsics>
          <projection>
            <p_fx>{fmt(intrinsics['fx'])}</p_fx>
            <p_fy>{fmt(intrinsics['fy'])}</p_fy>
            <p_cx>{fmt(intrinsics['cx'])}</p_cx>
            <p_cy>{fmt(intrinsics['cy'])}</p_cy>
            <tx>{fmt(tx)}</tx>
            <ty>0</ty>
          </projection>
        </lens>"""


@dataclass(frozen=True)
class SensorSpec:
    """One Gazebo camera, without the profile-sized intrinsics.

    ``layout_key`` selects the origin in ``sensor_layouts``. ``tx`` is the
    projection Tx: 0 for the left and color cameras, and ``-fx * baseline``
    for the right stereo camera.
    """

    name: str
    layout_key: str
    pixel_format: str
    topic: str
    info_topic: str
    optical_frame: str
    near: float
    far: float
    tx: float


def _camera_sensor(
    spec: SensorSpec,
    xyz: Vec3,
    intrinsics: Intrinsics,
    update_rate: float,
) -> str:
    """One Gazebo camera: pose, image size, clip planes, and topics."""
    pose = pose_text(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)
    hfov = hfov_from_intrinsics(intrinsics)
    return f"""      <sensor name="{spec.name}" type="camera">
        <pose>{pose}</pose>
        <gz_frame_id>{spec.optical_frame}</gz_frame_id>
        <topic>{spec.topic}</topic>
        <always_on>true</always_on>
        <update_rate>{rate_text(update_rate)}</update_rate>
        <visualize>true</visualize>
        <camera name="{spec.name}">
          <horizontal_fov>{fmt(hfov)}</horizontal_fov>
          <image>
            <width>{intrinsics['width']}</width>
            <height>{intrinsics['height']}</height>
            <format>{spec.pixel_format}</format>
          </image>
          <clip>
            <near>{fmt(spec.near)}</near>
            <far>{fmt(spec.far)}</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>{fmt(IMAGE_NOISE_STDDEV)}</stddev>
          </noise>
          <distortion>
            <k1>0</k1>
            <k2>0</k2>
            <k3>0</k3>
            <p1>0</p1>
            <p2>0</p2>
          </distortion>
{indent(_lens(intrinsics, spec.tx), 2)}
          <optical_frame_id>{spec.optical_frame}</optical_frame_id>
          <camera_info_topic>{spec.info_topic}</camera_info_topic>
        </camera>
      </sensor>"""


def _depth_sensor(xyz: Vec3, intrinsics: Intrinsics, optical_frame: str, update_rate: float) -> str:
    """Depth image registered to the color camera.

    Same pose, same intrinsics, same optical frame. That is what the real
    OAK-D publishes when depth is aligned to RGB, and what RTAB-Map assumes
    when rgb and depth share one camera_info.
    """
    pose = pose_text(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)
    hfov = hfov_from_intrinsics(intrinsics)
    return f"""      <sensor name="rgb_aligned_depth" type="depth_camera">
        <pose>{pose}</pose>
        <gz_frame_id>{optical_frame}</gz_frame_id>
        <topic>/camera/depth/image_raw</topic>
        <always_on>true</always_on>
        <update_rate>{rate_text(update_rate)}</update_rate>
        <visualize>true</visualize>
        <camera name="rgb_aligned_depth">
          <horizontal_fov>{fmt(hfov)}</horizontal_fov>
          <image>
            <width>{intrinsics['width']}</width>
            <height>{intrinsics['height']}</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>{fmt(DEPTH_CLIP_NEAR_M)}</near>
            <far>{fmt(DEPTH_CLIP_FAR_M)}</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>{fmt(IMAGE_NOISE_STDDEV)}</stddev>
          </noise>
{indent(_lens(intrinsics, 0.0), 2)}
          <optical_frame_id>{optical_frame}</optical_frame_id>
          <camera_info_topic>/camera/depth/camera_info</camera_info_topic>
        </camera>
      </sensor>"""


def _axis_noise(stddev: float, bias: float) -> str:
    """Gaussian noise for one IMU axis, including the bias stddev."""
    return f"""            <noise type="gaussian">
              <mean>0</mean>
              <stddev>{fmt(stddev)}</stddev>
              <bias_mean>0</bias_mean>
              <bias_stddev>{fmt(bias)}</bias_stddev>
            </noise>"""


def _imu_sensor(xyz: Vec3, imu_hz: float) -> str:
    """BNO086 IMU. The ENU reference keeps a tilted mount from looking level."""
    pose = pose_text(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)
    gyro = _axis_noise(gyro_stddev_rad_s(imu_hz), GYRO_BIAS_STDDEV_RAD_S)
    accel = _axis_noise(accel_stddev_m_s2(imu_hz), ACCEL_BIAS_STDDEV_M_S2)
    axes = "\n".join(f"          <{axis}>\n{gyro}\n          </{axis}>" for axis in "xyz")
    linear = "\n".join(f"          <{axis}>\n{accel}\n          </{axis}>" for axis in "xyz")
    # Without this element, gz-sim8 stores the spawn rotation as the IMU
    # reference (Imu.cc SetOrientationReference). A 17 deg mount then looks
    # level, and GravitySigma pulls the map the wrong way. ENU matches the
    # Gazebo world and REP-103. The localization child is what gz-sensors8
    # actually reads; the parent element is what makes Imu.cc replace the
    # spawn reference.
    return f"""      <sensor name="BNO086" type="imu">
        <pose>{pose}</pose>
        <gz_frame_id>{IMU_FRAME}</gz_frame_id>
        <topic>{IMU_TOPIC}</topic>
        <always_on>true</always_on>
        <update_rate>{rate_text(imu_hz)}</update_rate>
        <imu>
          <orientation_reference_frame>
            <localization>ENU</localization>
          </orientation_reference_frame>
          <angular_velocity>
{axes}
          </angular_velocity>
          <linear_acceleration>
{linear}
          </linear_acceleration>
        </imu>
      </sensor>"""


def _housing(mount: Mount) -> tuple[str, str, str]:
    """Inertial XML, visual/collision XML, and the SDF comment header."""
    spec = housing_spec()
    inertial = f"""      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>{fmt(spec.mass_kg)}</mass>
        <inertia>
          <ixx>{fmt(spec.ixx)}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{fmt(spec.iyy)}</iyy>
          <iyz>0</iyz>
          <izz>{fmt(spec.izz)}</izz>
        </inertia>
      </inertial>"""
    size = f"{fmt(spec.depth_m)} {fmt(spec.width_m)} {fmt(spec.height_m)}"
    visual = f"""      <visual name="housing">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
        <material>
          <ambient>{" ".join(fmt(channel) for channel in HOUSING_AMBIENT_RGBA)}</ambient>
          <diffuse>{" ".join(fmt(channel) for channel in HOUSING_DIFFUSE_RGBA)}</diffuse>
        </material>
      </visual>
      <collision name="housing">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
      </collision>"""
    header = (
        f"Generated from includes/gz/oakd_s2. Mount on base_link: {mount.pose_text()} "
        f"(pitch {mount.pitch_deg:g} deg down). Model name stays OakD-Lite so "
        f"PX4 x500_depth can include it."
    )
    return inertial, visual, header


def _stereo_sensors(layouts: dict[str, Vec3], profile: VisionProfile) -> str:
    """Left and right mono cameras, plus the IMU."""
    # One K for the pair. The right camera's own calibration is not used.
    shared = stereo_intrinsics(profile)
    right_tx = stereo_tx(float(shared["fx"]))
    specs = (
        SensorSpec(
            name="OV9282_left",
            layout_key="stereo_left",
            pixel_format="L8",
            topic="/camera/stereo/left/image_raw",
            info_topic="/camera/stereo/left/camera_info",
            optical_frame=STEREO_LEFT_OPTICAL,
            near=STEREO_CLIP_NEAR_M,
            far=STEREO_CLIP_FAR_M,
            tx=0.0,
        ),
        SensorSpec(
            name="OV9282_right",
            layout_key="stereo_right",
            pixel_format="L8",
            topic="/camera/stereo/right/image_raw",
            info_topic=RIGHT_INFO_IN,
            optical_frame=STEREO_RIGHT_OPTICAL,
            near=STEREO_CLIP_NEAR_M,
            far=STEREO_CLIP_FAR_M,
            tx=right_tx,
        ),
    )
    cameras = [
        _camera_sensor(spec, layouts[spec.layout_key], shared, profile.camera_hz) for spec in specs
    ]
    return "\n".join([*cameras, _imu_sensor(layouts["imu"], profile.imu_hz)])


def _rgbd_sensors(layouts: dict[str, Vec3], profile: VisionProfile) -> str:
    """Color camera, aligned depth, and the IMU."""
    color = color_intrinsics(profile)
    spec = SensorSpec(
        name="IMX378",
        layout_key="rgb",
        pixel_format="R8G8B8",
        topic="/camera/rgb/image_raw",
        info_topic="/camera/rgb/camera_info",
        optical_frame=RGB_OPTICAL,
        near=COLOR_CLIP_NEAR_M,
        far=COLOR_CLIP_FAR_M,
        tx=0.0,
    )
    return "\n".join(
        [
            _camera_sensor(spec, layouts[spec.layout_key], color, profile.camera_hz),
            _depth_sensor(layouts["rgb"], color, RGB_OPTICAL, profile.camera_hz),
            _imu_sensor(layouts["imu"], profile.imu_hz),
        ]
    )


def render_sdf(
    variant: str,
    mount: Mount | None = None,
    profile: VisionProfile | None = None,
) -> str:
    """Stereo or RGB-D SDF. An omitted profile is ``FULL_PROFILE``."""
    if variant not in ("stereo", "rgbd"):
        raise ValueError(variant)
    mount = mount or Mount()
    # Callers that omit the profile get FULL_PROFILE so the checked-in model
    # stays 1280x800. Container start passes profile_from_env(). An unset
    # VISION_PROFILE is cpu.
    profile = FULL_PROFILE if profile is None else profile
    layouts = sensor_layouts()
    inertial, visual, header = _housing(mount)
    if variant == "stereo":
        sensors = _stereo_sensors(layouts, profile)
    else:
        sensors = _rgbd_sensors(layouts, profile)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- {header} -->
<sdf version="1.9">
  <model name="OakD-Lite">
    <pose>0 0 0 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="{LINK_NAME}">
{inertial}
{visual}
{sensors}
      <gravity>true</gravity>
    </link>
  </model>
</sdf>
"""
