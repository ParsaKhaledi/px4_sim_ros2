#!/usr/bin/env python3
"""Render the OAK-D S2 SDF and URDF from includes/gz/oakd_s2/geometry.py.

Run with no arguments to refresh the model files and the x500 URDF from the
CAM_* environment variables. ``--urdf-only`` is what the state publisher
container runs. ``--patch-x500`` writes the same mount into PX4's x500_depth
model, which is the file Gazebo actually includes.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import geometry as geo


GZ_ROOT = Path(__file__).resolve().parent.parent
STEREO_SDF = GZ_ROOT / "models" / "OakD-Lite-stereo" / "model.sdf"
RGBD_SDF = GZ_ROOT / "models" / "OakD-Lite-rgbd" / "model.sdf"
URDF_PATH = GZ_ROOT / "x500_tf_publisher" / "x500_urdf.urdf"

POSE_RE = re.compile(r"(<pose\b[^>]*>)(.*?)(</pose>)", re.DOTALL)


def indent(text: str, spaces: int) -> str:
    pad = " " * spaces
    return "\n".join(pad + line if line else line for line in text.splitlines())


def fmt(value: float) -> str:
    """Stable, full-precision text. Calibration fx must survive a round trip."""
    return f"{value:.16g}"


def _lens(intrinsics: dict, tx: float) -> str:
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


def _camera_sensor(
    name: str,
    xyz,
    intrinsics: dict,
    pixel_format: str,
    topic: str,
    info_topic: str,
    optical_frame: str,
    near: float,
    far: float,
    tx: float,
) -> str:
    pose = geo.pose_text(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)
    hfov = geo.hfov_from_intrinsics(intrinsics)
    return f"""      <sensor name="{name}" type="camera">
        <pose>{pose}</pose>
        <gz_frame_id>{optical_frame}</gz_frame_id>
        <topic>{topic}</topic>
        <always_on>true</always_on>
        <update_rate>{geo.CAMERA_HZ}</update_rate>
        <visualize>true</visualize>
        <camera name="{name}">
          <horizontal_fov>{fmt(hfov)}</horizontal_fov>
          <image>
            <width>{intrinsics['width']}</width>
            <height>{intrinsics['height']}</height>
            <format>{pixel_format}</format>
          </image>
          <clip>
            <near>{fmt(near)}</near>
            <far>{fmt(far)}</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>{fmt(geo.IMAGE_NOISE_STDDEV)}</stddev>
          </noise>
          <distortion>
            <k1>0</k1>
            <k2>0</k2>
            <k3>0</k3>
            <p1>0</p1>
            <p2>0</p2>
          </distortion>
{indent(_lens(intrinsics, tx), 2)}
          <optical_frame_id>{optical_frame}</optical_frame_id>
          <camera_info_topic>{info_topic}</camera_info_topic>
        </camera>
      </sensor>"""


def _depth_sensor(xyz, intrinsics: dict, optical_frame: str) -> str:
    """Depth image registered to the color camera.

    Same pose, same intrinsics, same optical frame. That is what the real
    OAK-D publishes when depth is aligned to RGB, and what RTAB-Map assumes
    when rgb and depth share one camera_info.
    """
    pose = geo.pose_text(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)
    hfov = geo.hfov_from_intrinsics(intrinsics)
    return f"""      <sensor name="rgb_aligned_depth" type="depth_camera">
        <pose>{pose}</pose>
        <gz_frame_id>{optical_frame}</gz_frame_id>
        <topic>/camera/depth/image_raw</topic>
        <always_on>true</always_on>
        <update_rate>{geo.CAMERA_HZ}</update_rate>
        <visualize>true</visualize>
        <camera name="rgb_aligned_depth">
          <horizontal_fov>{fmt(hfov)}</horizontal_fov>
          <image>
            <width>{intrinsics['width']}</width>
            <height>{intrinsics['height']}</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.2</near>
            <far>12</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>{fmt(geo.IMAGE_NOISE_STDDEV)}</stddev>
          </noise>
{indent(_lens(intrinsics, 0.0), 2)}
          <optical_frame_id>{optical_frame}</optical_frame_id>
          <camera_info_topic>/camera/depth/camera_info</camera_info_topic>
        </camera>
      </sensor>"""


def _axis_noise(stddev: float, bias: float) -> str:
    return f"""            <noise type="gaussian">
              <mean>0</mean>
              <stddev>{fmt(stddev)}</stddev>
              <bias_mean>0</bias_mean>
              <bias_stddev>{fmt(bias)}</bias_stddev>
            </noise>"""


def _imu_sensor(xyz) -> str:
    pose = geo.pose_text(xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0)
    gyro = _axis_noise(geo.gyro_stddev_rad_s(), 1.0e-4)
    accel = _axis_noise(geo.accel_stddev_m_s2(), 1.0e-3)
    axes = "\n".join(f"          <{axis}>\n{gyro}\n          </{axis}>" for axis in "xyz")
    linear = "\n".join(f"          <{axis}>\n{accel}\n          </{axis}>" for axis in "xyz")
    return f"""      <sensor name="BNO086" type="imu">
        <pose>{pose}</pose>
        <gz_frame_id>{geo.IMU_FRAME}</gz_frame_id>
        <topic>{geo.IMU_TOPIC}</topic>
        <always_on>true</always_on>
        <update_rate>{geo.IMU_HZ}</update_rate>
        <imu>
          <angular_velocity>
{axes}
          </angular_velocity>
          <linear_acceleration>
{linear}
          </linear_acceleration>
        </imu>
      </sensor>"""


def _housing(mount: geo.Mount) -> tuple[str, str, str]:
    ixx, iyy, izz = geo.box_inertia(
        geo.MASS_KG, geo.HOUSING_DEPTH_M, geo.HOUSING_WIDTH_M, geo.HOUSING_HEIGHT_M
    )
    inertial = f"""      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>{fmt(geo.MASS_KG)}</mass>
        <inertia>
          <ixx>{fmt(ixx)}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{fmt(iyy)}</iyy>
          <iyz>0</iyz>
          <izz>{fmt(izz)}</izz>
        </inertia>
      </inertial>"""
    visual = f"""      <visual name="housing">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{fmt(geo.HOUSING_DEPTH_M)} {fmt(geo.HOUSING_WIDTH_M)} {fmt(geo.HOUSING_HEIGHT_M)}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.12 0.12 0.13 1</ambient>
          <diffuse>0.2 0.2 0.22 1</diffuse>
        </material>
      </visual>
      <collision name="housing">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{fmt(geo.HOUSING_DEPTH_M)} {fmt(geo.HOUSING_WIDTH_M)} {fmt(geo.HOUSING_HEIGHT_M)}</size>
          </box>
        </geometry>
      </collision>"""
    header = (
        f"Generated from includes/gz/oakd_s2. Mount on base_link: {mount.pose_text()} "
        f"(pitch {mount.pitch_deg:g} deg down). Model name stays OakD-Lite so "
        f"PX4 x500_depth can include it."
    )
    return inertial, visual, header


def render_sdf(variant: str, mount: geo.Mount | None = None) -> str:
    if variant not in ("stereo", "rgbd"):
        raise ValueError(variant)
    mount = mount or geo.Mount()
    layouts = geo.sensor_layouts()
    inertial, visual, header = _housing(mount)
    imu = _imu_sensor(layouts["imu"])
    if variant == "stereo":
        right_tx = geo.stereo_tx(geo.RIGHT_INTRINSICS["fx"])
        sensors = "\n".join(
            [
                _camera_sensor(
                    "OV9282_left",
                    layouts["stereo_left"],
                    geo.LEFT_INTRINSICS,
                    "L8",
                    "/camera/stereo/left/image_raw",
                    "/camera/stereo/left/camera_info",
                    geo.STEREO_LEFT_OPTICAL,
                    0.2,
                    30.0,
                    0.0,
                ),
                _camera_sensor(
                    "OV9282_right",
                    layouts["stereo_right"],
                    geo.RIGHT_INTRINSICS,
                    "L8",
                    "/camera/stereo/right/image_raw",
                    geo.RIGHT_INFO_IN,
                    geo.STEREO_RIGHT_OPTICAL,
                    0.2,
                    30.0,
                    right_tx,
                ),
                imu,
            ]
        )
    else:
        color = geo.color_intrinsics()
        sensors = "\n".join(
            [
                _camera_sensor(
                    "IMX378",
                    layouts["rgb"],
                    color,
                    "R8G8B8",
                    "/camera/rgb/image_raw",
                    "/camera/rgb/camera_info",
                    geo.RGB_OPTICAL,
                    0.08,
                    50.0,
                    0.0,
                ),
                _depth_sensor(layouts["rgb"], color, geo.RGB_OPTICAL),
                imu,
            ]
        )
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- {header} -->
<sdf version="1.9">
  <model name="OakD-Lite">
    <pose>0 0 0 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="{geo.LINK_NAME}">
{inertial}
{visual}
{sensors}
      <gravity>true</gravity>
    </link>
  </model>
</sdf>
"""


def _optical_joint(parent: str, optical: str) -> str:
    roll, pitch, yaw = geo.OPTICAL_RPY
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
    xyz,
    rpy=(0.0, 0.0, 0.0),
    include_link: bool = True,
) -> str:
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


def render_urdf(mount: geo.Mount | None = None) -> str:
    """URDF for robot_state_publisher.

    The mount pitch is on camera_joint, matching the x500_depth include pose.
    Each sensor then has a zero-rotation frame at its housing offset (the
    Gazebo sensor pose, x forward) and an optical child for image headers.
    Depth is aligned to the color optical frame, so it has no frame of its own.
    """
    mount = mount or geo.Mount()
    layouts = geo.sensor_layouts()
    ixx, iyy, izz = geo.box_inertia(
        geo.MASS_KG, geo.HOUSING_DEPTH_M, geo.HOUSING_WIDTH_M, geo.HOUSING_HEIGHT_M
    )
    size = f"{fmt(geo.HOUSING_DEPTH_M)} {fmt(geo.HOUSING_WIDTH_M)} {fmt(geo.HOUSING_HEIGHT_M)}"
    parts = [
        '<?xml version="1.0"?>',
        "<!-- Generated from includes/gz/oakd_s2. "
        f"camera_joint matches PX4 x500_depth: {mount.pose_text()}. -->",
        '<robot name="x500_depth">',
        '  <link name="base_link"/>',
        f"""  <link name="{geo.LINK_NAME}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{fmt(geo.MASS_KG)}"/>
      <inertia ixx="{fmt(ixx)}" ixy="0" ixz="0" iyy="{fmt(iyy)}" iyz="0" izz="{fmt(izz)}"/>
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
            geo.BASE_FRAME,
            geo.LINK_NAME,
            (mount.x, mount.y, mount.z),
            (0.0, mount.pitch_rad, 0.0),
            include_link=False,
        ),
    ]
    # camera_joint's child link is declared above, so drop the empty link
    # that _fixed_joint would have appended. Done via replace on that one call.

    def sensor_frames(name: str, xyz, optical: str) -> str:
        frame = name + "_frame"
        block = _fixed_joint(name + "_joint", geo.LINK_NAME, frame, xyz)
        block += _optical_joint(frame, optical)
        return block

    parts.append(sensor_frames("stereo_left_camera", layouts["stereo_left"], geo.STEREO_LEFT_OPTICAL))
    parts.append(sensor_frames("stereo_right_camera", layouts["stereo_right"], geo.STEREO_RIGHT_OPTICAL))
    parts.append(sensor_frames("camera_rgb", layouts["rgb"], geo.RGB_OPTICAL))
    parts.append(_fixed_joint("imu_joint", geo.LINK_NAME, geo.IMU_FRAME, layouts["imu"]))
    parts.append("</robot>\n")
    return "\n".join(parts)


def _replace_first_pose(block: str, pose: str) -> str:
    if POSE_RE.search(block):
        return POSE_RE.sub(lambda match: match.group(1) + pose + match.group(3), block, count=1)
    close = block.rfind("</")
    if close < 0:
        raise ValueError("no closing tag to attach a pose to")
    return block[:close] + f"<pose>{pose}</pose>\n" + block[close:]


def _block_span(xml: str, start_tag: str, end_tag: str, needle: str) -> tuple[int, int]:
    cursor = 0
    while True:
        start = xml.find(start_tag, cursor)
        if start < 0:
            raise ValueError(f"no {start_tag} containing {needle}")
        end = xml.find(end_tag, start)
        if end < 0:
            raise ValueError(f"unclosed {start_tag}")
        end += len(end_tag)
        if needle in xml[start:end]:
            return start, end
        cursor = end


def patch_x500_depth_xml(xml: str, mount: geo.Mount | None = None) -> str:
    """Set the OakD-Lite include pose and the camera_link joint to ``mount``.

    The rest of the PX4 model is left as text. Only those two pose values change,
    so a Gazebo camera and the URDF camera_joint describe the same bracket.
    """
    mount = mount or geo.Mount()
    pose = mount.pose_text()
    start, end = _block_span(xml, "<include", "</include>", "OakD-Lite")
    xml = xml[:start] + _replace_first_pose(xml[start:end], pose) + xml[end:]
    start, end = _block_span(xml, "<joint", "</joint>", "camera_link")
    xml = xml[:start] + _replace_first_pose(xml[start:end], pose) + xml[end:]
    return xml


def patch_x500_depth_file(path: Path, mount: geo.Mount | None = None) -> str:
    file_path = Path(path)
    original = file_path.read_text(encoding="utf-8")
    updated = patch_x500_depth_xml(original, mount)
    file_path.write_text(updated, encoding="utf-8")
    return (mount or geo.Mount()).pose_text()


def write_models(mount: geo.Mount | None = None, urdf_only: bool = False) -> None:
    mount = mount or geo.mount_from_env()
    URDF_PATH.write_text(render_urdf(mount), encoding="utf-8")
    if urdf_only:
        return
    STEREO_SDF.write_text(render_sdf("stereo", mount), encoding="utf-8")
    RGBD_SDF.write_text(render_sdf("rgbd", mount), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render the OAK-D S2 SDF and URDF.")
    parser.add_argument("--urdf-only", action="store_true")
    parser.add_argument("--patch-x500", type=Path, default=None)
    args = parser.parse_args(argv)
    mount = geo.mount_from_env()
    if args.patch_x500:
        pose = patch_x500_depth_file(args.patch_x500, mount)
        print(f"x500_depth camera pose set to {pose}")
        return 0
    write_models(mount, urdf_only=args.urdf_only)
    what = "URDF" if args.urdf_only else "stereo SDF, RGB-D SDF, and URDF"
    print(f"Wrote {what} for mount {mount.pose_text()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
