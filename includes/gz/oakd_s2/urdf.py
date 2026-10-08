"""URDF text for robot_state_publisher.

The mount pitch is on ``camera_joint``. Each sensor has a frame at its
housing offset and an optical child for image headers.
"""

from __future__ import annotations

from frames import (
    BASE_FRAME,
    IMU_FRAME,
    LINK_NAME,
    OPTICAL_RPY,
    RGB_OPTICAL,
    STEREO_LEFT_OPTICAL,
    STEREO_RIGHT_OPTICAL,
    Mount,
    sensor_layouts,
)
from hardware import housing_spec
from sdf import fmt

Vec3 = tuple[float, float, float]


def _optical_joint(parent: str, optical: str) -> str:
    """Fixed joint from a sensor frame into its optical child."""
    roll, pitch, yaw = OPTICAL_RPY
    rpy_text = f"{roll:.8f} {pitch:.8f} {yaw:.8f}"
    return f"""  <joint name="{optical}_joint" type="fixed">
    <parent link="{parent}"/>
    <child link="{optical}"/>
    <origin xyz="0 0 0" rpy="{rpy_text}"/>
  </joint>
  <link name="{optical}"/>
"""


def _fixed_joint(
    name: str,
    parent: str,
    child: str,
    xyz: Vec3,
    rpy: Vec3 = (0.0, 0.0, 0.0),
    include_link: bool = True,
) -> str:
    """Fixed joint. The child link is omitted when ``include_link`` is false."""
    xyz_text = " ".join(f"{v:.6f}" for v in xyz)
    rpy_text = " ".join(f"{v:.6f}" for v in rpy)
    joint = f"""  <joint name="{name}" type="fixed">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{xyz_text}" rpy="{rpy_text}"/>
  </joint>
"""
    if include_link:
        joint += f'  <link name="{child}"/>\n'
    return joint


def render_urdf(mount: Mount | None = None) -> str:
    """URDF for robot_state_publisher.

    The mount pitch is on camera_joint, matching the x500_depth include pose.
    Each sensor then has a zero-rotation frame at its housing offset (the
    Gazebo sensor pose, x forward) and an optical child for image headers.
    Depth is aligned to the color optical frame, so it has no frame of its own.
    """
    mount = mount or Mount()
    layouts = sensor_layouts()
    spec = housing_spec()
    size = f"{fmt(spec.depth_m)} {fmt(spec.width_m)} {fmt(spec.height_m)}"
    parts = [
        '<?xml version="1.0"?>',
        "<!-- Generated from includes/gz/oakd_s2. "
        f"camera_joint matches PX4 x500_depth pose {mount.pose_text()}. -->",
        '<robot name="x500_depth">',
        '  <link name="base_link"/>',
        f"""  <link name="{LINK_NAME}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{fmt(spec.mass_kg)}"/>
      <inertia ixx="{fmt(spec.ixx)}" ixy="0" ixz="0" iyy="{fmt(spec.iyy)}" iyz="0" izz="{fmt(spec.izz)}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{size}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{size}"/>
      </geometry>
    </collision>
  </link>""",
        _fixed_joint(
            "camera_joint",
            BASE_FRAME,
            LINK_NAME,
            (mount.x, mount.y, mount.z),
            (0.0, mount.pitch_rad, 0.0),
            include_link=False,
        ),
    ]
    # camera_joint's child link is declared above, so include_link=False
    # drops the empty link _fixed_joint would have appended.

    def sensor_frames(name: str, xyz: Vec3, optical: str) -> str:
        """Sensor frame at the housing offset, plus its optical child."""
        frame = name + "_frame"
        block = _fixed_joint(name + "_joint", LINK_NAME, frame, xyz)
        block += _optical_joint(frame, optical)
        return block

    parts.append(sensor_frames("stereo_left_camera", layouts["stereo_left"], STEREO_LEFT_OPTICAL))
    parts.append(sensor_frames("stereo_right_camera", layouts["stereo_right"], STEREO_RIGHT_OPTICAL))
    parts.append(sensor_frames("camera_rgb", layouts["rgb"], RGB_OPTICAL))
    parts.append(_fixed_joint("imu_joint", LINK_NAME, IMU_FRAME, layouts["imu"]))
    parts.append("</robot>\n")
    return "\n".join(parts)
