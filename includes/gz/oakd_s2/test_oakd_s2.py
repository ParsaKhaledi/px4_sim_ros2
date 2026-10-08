#!/usr/bin/env python3
"""Pose, baseline, and SLAM-gate checks that do not need Gazebo or ROS."""

from __future__ import annotations

import importlib.util
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import geometry as geo  # noqa: E402
import render_oakd as render  # noqa: E402
import rtabmap_params as rtab  # noqa: E402


def load_module(path: Path, name: str):
    """Import a file by path so the health log and eval script can be tested."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolve the class module through sys.modules during exec.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


REPO = HERE.parents[2]
HEALTH = load_module(REPO / "HealthCheck" / "rtabmap_health_log.py", "rtabmap_health_log")
EVAL = load_module(REPO / "scripts" / "eval_slam_accuracy.py", "eval_slam_accuracy")
PROBE = load_module(REPO / "HealthCheck" / "vision_rate_probe.py", "vision_rate_probe")


def pose_values(text: str):
    """Split an SDF pose string into floats."""
    return [float(item) for item in text.split()]


def local(tag: str) -> str:
    """XML tag name without a namespace."""
    return tag.rsplit("}", 1)[-1]


def findall(root, name: str):
    """Elements whose local tag name is ``name``."""
    return [node for node in root.iter() if local(node.tag) == name]


def sensor_pose(root, sensor_name: str):
    """Pose floats of the sensor named ``sensor_name``."""
    for node in findall(root, "sensor"):
        if node.attrib.get("name") == sensor_name:
            pose = next(child for child in node if local(child.tag) == "pose")
            return pose_values(pose.text)
    raise KeyError(sensor_name)


def child_text(node, name: str) -> str:
    """Text of the first child named ``name``, or an empty string."""
    for child in node:
        if local(child.tag) == name and child.text:
            return child.text.strip()
    return ""


class GeometryTest(unittest.TestCase):
    """Intrinsics, profiles, and the warnings that do not need a render."""

    def test_stereo_tx_uses_the_shared_left_fx(self):
        """Tx uses the shared left fx and the 7.5 cm baseline."""
        shared = geo.stereo_tx(geo.LEFT_INTRINSICS["fx"])
        self.assertAlmostEqual(shared, -geo.LEFT_INTRINSICS["fx"] * 0.075, places=9)
        self.assertLess(shared, 0.0)
        # The unused right calibration would have produced a different Tx.
        self.assertNotAlmostEqual(shared, geo.stereo_tx(geo.RIGHT_INTRINSICS["fx"]), places=3)
        full = geo.stereo_intrinsics(geo.FULL_PROFILE)
        left_p = geo.rectified_p(right=False)
        right_p = geo.rectified_p(right=True)
        self.assertEqual(geo.rectified_k()[0], left_p[0])
        self.assertEqual(left_p[0], right_p[0])
        self.assertEqual(left_p[0], full["fx"])
        self.assertEqual(left_p[2], right_p[2])
        self.assertEqual(left_p[5], right_p[5])
        self.assertEqual(left_p[6], right_p[6])
        self.assertEqual(left_p[3], 0.0)
        self.assertAlmostEqual(right_p[3], geo.stereo_tx(full["fx"]), places=9)
        self.assertAlmostEqual(right_p[3] / right_p[0], -0.075, places=12)
        self.assertAlmostEqual(right_p[3], shared * 2.0, places=9)

    def test_corrected_projection_fills_zero_tx_once(self):
        """A zero Tx is replaced once and left alone on a second pass."""
        fx = geo.RIGHT_INTRINSICS["fx"]
        k = [fx, 0.0, 10.0, 0.0, fx, 20.0, 0.0, 0.0, 1.0]
        zeros = [0.0] * 12
        filled = geo.corrected_projection(zeros, k)
        self.assertAlmostEqual(filled[0], fx)
        self.assertAlmostEqual(filled[3], geo.stereo_tx(fx))
        again = geo.corrected_projection(filled, k)
        self.assertEqual(again[3], filled[3])

    def test_corrected_projection_rejects_missing_fx(self):
        """A projection with no recoverable fx is rejected."""
        self.assertIsNone(geo.corrected_projection([0.0] * 12, [0.0] * 9))

    def test_color_fov_matches_af_datasheet(self):
        """The color pinhole matches the auto-focus datasheet HFOV."""
        color = geo.color_intrinsics()
        hfov = math.degrees(geo.hfov_from_intrinsics(color))
        vfov = math.degrees(geo.vfov_from_intrinsics(color))
        self.assertAlmostEqual(hfov, 66.0, places=6)
        self.assertAlmostEqual(color["fx"], color["fy"], places=6)
        self.assertAlmostEqual(vfov, 52.0, delta=0.15)
        self.assertAlmostEqual(geo.diagonal_fov_deg(hfov, vfov), 78.0, delta=0.2)

    def test_calibration_hfov_is_not_silently_replaced(self):
        """The stereo HFOV stays the calibration, not the 80 deg nominal."""
        hfov = math.degrees(geo.hfov_from_intrinsics(geo.LEFT_INTRINSICS))
        self.assertAlmostEqual(hfov, 76.4, delta=0.1)
        self.assertLess(hfov, 80.0)

    def test_optical_axes_and_baseline(self):
        """The optical axes and the right-in-left baseline match the housing."""
        rotation = geo.optical_rotation()
        self.assertAlmostEqual(geo.matvec(rotation, (0.0, 0.0, 1.0))[0], 1.0, places=6)
        baseline = geo.right_in_left_optical()
        self.assertAlmostEqual(baseline[0], geo.BASELINE_M, places=6)
        self.assertAlmostEqual(baseline[1], 0.0, places=6)
        self.assertAlmostEqual(baseline[2], 0.0, places=6)

    def test_positive_pitch_points_down(self):
        """A positive pitch rotates the glass downward."""
        forward = geo.matvec(geo.rot_y(math.radians(17.0)), (1.0, 0.0, 0.0))
        self.assertLess(forward[2], 0.0)

    def test_intrinsics_scale_keeps_fov_and_baseline(self):
        """Scaling the image keeps the field of view and the baseline ratio."""
        full = geo.stereo_intrinsics(geo.FULL_PROFILE)
        cpu = geo.stereo_intrinsics(geo.CPU_PROFILE)
        hw = geo.stereo_intrinsics(geo.HW_PROFILE)
        self.assertEqual((full["width"], full["height"]), (1280, 800))
        self.assertEqual((hw["width"], hw["height"]), (1280, 800))
        self.assertEqual((cpu["width"], cpu["height"]), (320, 200))
        self.assertAlmostEqual(full["fx"], geo.LEFT_INTRINSICS["fx"] * 2.0, places=9)
        self.assertAlmostEqual(cpu["fx"], geo.LEFT_INTRINSICS["fx"] * 0.5, places=9)
        self.assertAlmostEqual(cpu["cx"], geo.LEFT_INTRINSICS["cx"] * 0.5, places=9)
        self.assertEqual(hw["fx"], full["fx"])
        for profile in (geo.FULL_PROFILE, geo.CPU_PROFILE, geo.HW_PROFILE):
            src = geo.stereo_intrinsics(profile)
            left = geo.rectified_p(False, profile=profile)
            right = geo.rectified_p(True, profile=profile)
            shared = geo.rectified_k(profile)
            self.assertEqual(shared[0], left[0])
            self.assertEqual(shared[0], right[0])
            self.assertEqual(left[3], 0.0)
            self.assertAlmostEqual(right[3], -src["fx"] * geo.BASELINE_M, places=9)
            self.assertAlmostEqual(right[3] / right[0], -geo.BASELINE_M, places=9)
            self.assertNotEqual(right[3], geo.stereo_tx(geo.RIGHT_INTRINSICS["fx"]))
            self.assertAlmostEqual(geo.hfov_from_intrinsics(src), geo.hfov_from_intrinsics(geo.LEFT_INTRINSICS), places=9)
            self.assertAlmostEqual(geo.vfov_from_intrinsics(src), geo.vfov_from_intrinsics(geo.LEFT_INTRINSICS), places=9)
        color_full = geo.color_intrinsics(geo.FULL_PROFILE)
        color_cpu = geo.color_intrinsics(geo.CPU_PROFILE)
        color_hw = geo.color_intrinsics(geo.HW_PROFILE)
        self.assertEqual((color_cpu["width"], color_cpu["height"]), (320, 240))
        self.assertEqual((color_hw["width"], color_hw["height"]), (640, 480))
        self.assertAlmostEqual(geo.hfov_from_intrinsics(color_full), geo.hfov_from_intrinsics(color_cpu), places=9)
        self.assertAlmostEqual(geo.vfov_from_intrinsics(color_full), geo.vfov_from_intrinsics(color_cpu), places=9)
        self.assertEqual(color_hw["fx"], color_full["fx"])

    def test_profile_overrides_win_over_cpu_defaults(self):
        """CAM_* and IMU_RATE_HZ replace the cpu profile fields."""
        cpu = geo.profile_from_env({"VISION_PROFILE": "cpu"})
        self.assertEqual((cpu.stereo_width, cpu.stereo_height), (320, 200))
        self.assertEqual((cpu.color_width, cpu.color_height), (320, 240))
        self.assertEqual(cpu.camera_hz, 10)
        self.assertEqual(cpu.imu_hz, 100)
        mixed = geo.profile_from_env(
            {
                "VISION_PROFILE": "cpu",
                "CAM_STEREO_RES": "640x400",
                "CAM_RATE_HZ": "8",
                "CAM_COLOR_HEIGHT": "",
            }
        )
        self.assertEqual((mixed.stereo_width, mixed.stereo_height), (640, 400))
        self.assertEqual(mixed.camera_hz, 8)
        self.assertEqual(mixed.color_height, 240)
        self.assertEqual(mixed.imu_hz, 100)
        unset = geo.profile_from_env({})
        self.assertEqual(unset.name, "cpu")
        self.assertEqual((unset.stereo_width, unset.stereo_height), (320, 200))
        self.assertEqual(unset.camera_hz, 10)
        self.assertEqual(unset.imu_hz, 100)
        for raw in ("", "   ", None):
            empty = geo.profile_from_env({"VISION_PROFILE": raw})
            self.assertEqual(empty.name, "cpu")
            self.assertEqual((empty.stereo_width, empty.stereo_height), (320, 200))
            self.assertEqual(empty.camera_hz, 10)
        full = geo.profile_from_env({"VISION_PROFILE": "full"})
        self.assertEqual(full.name, "full")
        self.assertEqual((full.stereo_width, full.stereo_height), (1280, 800))
        self.assertEqual(full.camera_hz, 30)
        hw = geo.profile_from_env({"VISION_PROFILE": "HW"})
        self.assertEqual(hw.name, "hw")
        self.assertEqual((hw.stereo_width, hw.stereo_height), (1280, 800))
        self.assertEqual((hw.color_width, hw.color_height), (640, 480))
        self.assertEqual(hw.camera_hz, 15)
        self.assertEqual(hw.imu_hz, 200)
        with self.assertRaises(ValueError):
            geo.profile_from_env({"VISION_PROFILE": "gpu"})

    def test_stereo_override_accepts_only_calibration_scales(self):
        """Only the 16:10 calibration scales are accepted as stereo sizes."""
        for width, height in geo.ALLOWED_STEREO_RESOLUTIONS:
            chosen = geo.profile_from_env(
                {"VISION_PROFILE": "hw", "CAM_STEREO_WIDTH": str(width), "CAM_STEREO_HEIGHT": str(height)}
            )
            self.assertEqual((chosen.stereo_width, chosen.stereo_height), (width, height))
            src = geo.stereo_intrinsics(chosen)
            scale = width / geo.LEFT_INTRINSICS["width"]
            self.assertAlmostEqual(src["fx"], geo.LEFT_INTRINSICS["fx"] * scale, places=9)
            self.assertAlmostEqual(src["fy"], geo.LEFT_INTRINSICS["fy"] * scale, places=9)
            self.assertAlmostEqual(src["cx"], geo.LEFT_INTRINSICS["cx"] * scale, places=9)
            self.assertAlmostEqual(src["cy"], geo.LEFT_INTRINSICS["cy"] * scale, places=9)
            right = geo.rectified_p(True, profile=chosen)
            self.assertAlmostEqual(right[3] / right[0], -0.075, places=12)
            self.assertEqual(geo.rectified_p(False, profile=chosen)[3], 0.0)
        agreed = geo.profile_from_env(
            {"CAM_STEREO_RES": "320x200", "CAM_STEREO_WIDTH": "320", "CAM_STEREO_HEIGHT": "200"}
        )
        self.assertEqual((agreed.stereo_width, agreed.stereo_height), (320, 200))
        for bad in ("1280x720", "160x100", "800x800", "1280", "640x200", "0x800"):
            with self.assertRaises(ValueError) as caught:
                geo.profile_from_env({"VISION_PROFILE": "full", "CAM_STEREO_RES": bad})
            self.assertIn("1280x800", str(caught.exception))
        with self.assertRaises(ValueError) as caught:
            geo.profile_from_env({"VISION_PROFILE": "cpu", "CAM_STEREO_WIDTH": "160"})
        self.assertIn("160x200", str(caught.exception))
        with self.assertRaises(ValueError) as caught:
            geo.profile_from_env(
                {"CAM_STEREO_RES": "1280x800", "CAM_STEREO_WIDTH": "640", "CAM_STEREO_HEIGHT": "400"}
            )
        self.assertIn("disagrees", str(caught.exception))
        with self.assertRaises(ValueError):
            geo.profile_from_env({"CAM_RATE_HZ": "0"})

    def test_sub_hd_warning_is_only_for_full_and_hw(self):
        """The below-HD warning is for full and hw, not cpu."""
        self.assertIsNone(geo.sub_hd_warning(geo.FULL_PROFILE))
        self.assertIsNone(geo.sub_hd_warning(geo.HW_PROFILE))
        self.assertIsNone(geo.sub_hd_warning(geo.CPU_PROFILE))
        low = geo.profile_from_env({"VISION_PROFILE": "full", "CAM_STEREO_RES": "640x400"})
        warning = geo.sub_hd_warning(low)
        self.assertIn("below 1280x720", warning)
        self.assertIn("VISION_PROFILE=full", warning)
        hw_low = geo.profile_from_env({"VISION_PROFILE": "hw", "CAM_STEREO_RES": "320x200"})
        self.assertIn("VISION_PROFILE=hw", geo.sub_hd_warning(hw_low))
        cpu_native = geo.profile_from_env({"VISION_PROFILE": "cpu", "CAM_STEREO_RES": "1280x800"})
        self.assertIsNone(geo.sub_hd_warning(cpu_native))

    def test_no_gpu_warning_is_only_for_full_and_hw(self):
        """The missing-GPU warning is for full and hw, and only from the SDF render."""
        self.assertFalse(geo.gpu_visible(dri_nodes=[], nvidia_ok=False))
        self.assertTrue(geo.gpu_visible(dri_nodes=["/dev/dri/renderD128"], nvidia_ok=False))
        self.assertTrue(geo.gpu_visible(dri_nodes=[], nvidia_ok=True))
        for profile in (geo.FULL_PROFILE, geo.HW_PROFILE):
            warning = geo.no_gpu_warning(profile, gpu_present=False)
            self.assertIn("no GPU", warning)
            self.assertIn(f"VISION_PROFILE={profile.name}", warning)
            self.assertIn("Set VISION_PROFILE=cpu", warning)
            self.assertIsNone(geo.no_gpu_warning(profile, gpu_present=True))
            # RTAB-Map does not render. The warning is the PX4 SDF render only.
            lines = rtab.startup_log_lines(profile, "stereo")
            self.assertFalse(any("no GPU" in line for line in lines))
            rendered = render.gazebo_startup_warnings(profile, write_sdf=True, gpu_present=False)
            self.assertEqual(sum("no GPU" in line for line in rendered), 1)
            urdf_only = render.gazebo_startup_warnings(profile, write_sdf=False, gpu_present=False)
            self.assertFalse(any("no GPU" in line for line in urdf_only))
        self.assertIsNone(geo.no_gpu_warning(geo.CPU_PROFILE, gpu_present=False))
        cpu_lines = rtab.startup_log_lines(geo.CPU_PROFILE, "stereo")
        self.assertFalse(any("no GPU" in line for line in cpu_lines))
        cpu_render = render.gazebo_startup_warnings(geo.CPU_PROFILE, write_sdf=True, gpu_present=False)
        self.assertFalse(any("no GPU" in line for line in cpu_render))
        relay = (HERE / "stereo_info_relay.py").read_text(encoding="utf-8")
        self.assertNotIn("no_gpu_warning", relay)


class RenderTest(unittest.TestCase):
    """SDF and URDF text for the three profiles and the x500 pose patch."""

    def setUp(self):
        """Mounts the render tests walk: level, default, pitched, and shifted."""
        self.mounts = [
            geo.Mount(pitch_deg=0.0),
            geo.Mount(),
            geo.Mount(pitch_deg=30.0),
            geo.Mount(x=0.2, y=-0.01, z=0.3, pitch_deg=-10.0),
        ]

    def test_sensor_poses_match_across_pitches(self):
        """Sensor poses follow the mount at every pitch."""
        layouts = geo.sensor_layouts()
        for mount in self.mounts:
            stereo = ET.fromstring(render.render_sdf("stereo", mount))
            rgbd = ET.fromstring(render.render_sdf("rgbd", mount))
            urdf = ET.fromstring(render.render_urdf(mount))
            joints = {node.attrib["name"]: node for node in findall(urdf, "joint")}

            left = sensor_pose(stereo, "OV9282_left")
            right = sensor_pose(stereo, "OV9282_right")
            self.assertEqual(left[3:], [0.0, 0.0, 0.0])
            self.assertAlmostEqual(left[1] - right[1], geo.BASELINE_M, places=6)
            self.assertAlmostEqual(left[0], layouts["stereo_left"][0], places=6)
            self.assertAlmostEqual(left[1], layouts["stereo_left"][1], places=6)

            for sdf_name, joint_name, key in (
                ("OV9282_left", "stereo_left_camera_joint", "stereo_left"),
                ("OV9282_right", "stereo_right_camera_joint", "stereo_right"),
            ):
                sdf_xyz = sensor_pose(stereo, sdf_name)[:3]
                urdf_xyz = [float(v) for v in joints[joint_name].find("origin").attrib["xyz"].split()]
                self.assertEqual(sdf_xyz, [round(v, 6) for v in layouts[key]])
                for got, expect in zip(urdf_xyz, layouts[key]):
                    self.assertAlmostEqual(got, expect, places=5)

            color = geo.color_intrinsics()
            rgb_pose = sensor_pose(rgbd, "IMX378")
            depth_pose = sensor_pose(rgbd, "rgb_aligned_depth")
            self.assertEqual(rgb_pose[:3], depth_pose[:3])
            self.assertEqual(child_text(self._sensor(rgbd, "rgb_aligned_depth"), "gz_frame_id"), geo.RGB_OPTICAL)
            self.assertEqual(child_text(self._sensor(rgbd, "IMX378"), "gz_frame_id"), geo.RGB_OPTICAL)
            self.assertAlmostEqual(float(self._nested(rgbd, "IMX378", "fx")), color["fx"], places=4)
            self.assertAlmostEqual(float(self._nested(rgbd, "rgb_aligned_depth", "fx")), color["fx"], places=4)

            cam = joints["camera_joint"].find("origin")
            xyz = [float(v) for v in cam.attrib["xyz"].split()]
            rpy = [float(v) for v in cam.attrib["rpy"].split()]
            self.assertEqual(xyz, [mount.x, mount.y, mount.z])
            self.assertAlmostEqual(rpy[1], mount.pitch_rad, places=5)
            self.assertEqual(rpy[0], 0.0)
            self.assertEqual(rpy[2], 0.0)

            # Pitch lives on the mount, not on the individual sensors.
            sensor_rpy = [float(v) for v in joints["stereo_left_camera_joint"].find("origin").attrib["rpy"].split()]
            self.assertEqual(sensor_rpy, [0.0, 0.0, 0.0])

            patched = render.patch_x500_depth_xml(self.x500_sample(), mount)
            patched_root = ET.fromstring(patched)
            poses = []
            for include in findall(patched_root, "include"):
                if "OakD-Lite" in "".join(include.itertext()):
                    poses.append(pose_values(child_text(include, "pose")))
            joint = next(node for node in findall(patched_root, "joint") if "camera_link" in "".join(node.itertext()))
            poses.append(pose_values(child_text(joint, "pose")))
            self.assertEqual(len(poses), 2)
            for pose in poses:
                self.assertAlmostEqual(pose[0], mount.x, places=5)
                self.assertAlmostEqual(pose[1], mount.y, places=5)
                self.assertAlmostEqual(pose[2], mount.z, places=5)
                self.assertAlmostEqual(pose[4], mount.pitch_rad, places=5)
            self.assertIn("model://x500", patched)

    def test_frames_intrinsics_and_noise(self):
        """Frames, intrinsics, clip, and noise in the rendered SDF match the constants."""
        stereo = ET.fromstring(render.render_sdf("stereo", geo.Mount()))
        links = {node.attrib["name"] for node in findall(ET.fromstring(render.render_urdf(geo.Mount())), "link")}
        for sensor_name, frame in (
            ("OV9282_left", geo.STEREO_LEFT_OPTICAL),
            ("OV9282_right", geo.STEREO_RIGHT_OPTICAL),
            ("BNO086", geo.IMU_FRAME),
        ):
            sensor = self._sensor(stereo, sensor_name)
            self.assertIn(child_text(sensor, "gz_frame_id"), links)
        full_k = geo.stereo_intrinsics(geo.FULL_PROFILE)
        right_tx = float(self._nested(stereo, "OV9282_right", "tx"))
        self.assertAlmostEqual(right_tx, geo.stereo_tx(full_k["fx"]), places=4)
        self.assertAlmostEqual(right_tx / full_k["fx"], -0.075, places=6)
        self.assertEqual(self._nested(stereo, "OV9282_left", "width"), "1280")
        self.assertEqual(self._nested(stereo, "OV9282_left", "height"), "800")
        self.assertAlmostEqual(float(self._nested(stereo, "OV9282_left", "tx")), 0.0, places=6)
        for field in ("fx", "fy", "cx", "cy"):
            self.assertAlmostEqual(
                float(self._nested(stereo, "OV9282_left", field)),
                float(self._nested(stereo, "OV9282_right", field)),
                places=6,
            )
            self.assertAlmostEqual(
                float(self._nested(stereo, "OV9282_left", field)),
                full_k[field],
                places=4,
            )
        self.assertGreater(float(self._nested(stereo, "OV9282_left", "stddev")), 0.0)
        self.assertEqual(child_text(self._sensor(stereo, "BNO086"), "update_rate"), str(geo.IMU_HZ))
        for variant in ("stereo", "rgbd"):
            model = ET.fromstring(render.render_sdf(variant, geo.Mount()))
            imu = self._sensor(model, "BNO086")
            localization = [
                node.text.strip()
                for node in imu.iter()
                if local(node.tag) == "localization" and node.text
            ]
            self.assertEqual(localization, ["ENU"])
        self.assertIn(geo.LINK_NAME, {node.attrib.get("name") for node in findall(stereo, "link")})

    def test_cpu_profile_sdf_scales_and_keeps_model_names(self):
        """The cpu SDF is the scaled image and keeps the OakD-Lite name."""
        stereo = ET.fromstring(render.render_sdf("stereo", geo.Mount(), geo.CPU_PROFILE))
        rgbd = ET.fromstring(render.render_sdf("rgbd", geo.Mount(), geo.CPU_PROFILE))
        self.assertIn("OakD-Lite", {node.attrib.get("name") for node in findall(stereo, "model")})
        self.assertIn(geo.LINK_NAME, {node.attrib.get("name") for node in findall(stereo, "link")})
        self.assertEqual(self._nested(stereo, "OV9282_left", "width"), "320")
        self.assertEqual(self._nested(stereo, "OV9282_left", "height"), "200")
        self.assertEqual(self._nested(stereo, "OV9282_right", "width"), "320")
        self.assertEqual(self._nested(stereo, "OV9282_left", "update_rate"), "10")
        self.assertEqual(self._nested(stereo, "BNO086", "update_rate"), "100")
        fx = geo.stereo_intrinsics(geo.CPU_PROFILE)["fx"]
        self.assertAlmostEqual(float(self._nested(stereo, "OV9282_left", "fx")), fx, places=6)
        self.assertAlmostEqual(float(self._nested(stereo, "OV9282_right", "fx")), fx, places=6)
        self.assertAlmostEqual(float(self._nested(stereo, "OV9282_right", "tx")), geo.stereo_tx(fx), places=6)
        full = ET.fromstring(render.render_sdf("stereo", geo.Mount(), geo.FULL_PROFILE))
        self.assertAlmostEqual(
            float(self._nested(full, "OV9282_left", "horizontal_fov")),
            float(self._nested(stereo, "OV9282_left", "horizontal_fov")),
            places=6,
        )
        self.assertEqual(self._nested(rgbd, "IMX378", "width"), "320")
        self.assertEqual(self._nested(rgbd, "IMX378", "height"), "240")
        self.assertEqual(self._nested(rgbd, "rgb_aligned_depth", "width"), "320")
        self.assertEqual(self._nested(rgbd, "rgb_aligned_depth", "height"), "240")
        self.assertEqual(self._nested(rgbd, "IMX378", "update_rate"), "10")
        # The URDF has the mount and the frames. Image size and rate are SDF-only,
        # so the full-profile URDF stays the checked-in file.
        self.assertIn(geo.LINK_NAME, render.render_urdf(geo.Mount()))

    def test_hw_sdf_matches_full_stereo_at_a_lower_rate(self):
        """hw matches full stereo size at the lower hw rate."""
        full = ET.fromstring(render.render_sdf("stereo", geo.Mount(), geo.FULL_PROFILE))
        hw = ET.fromstring(render.render_sdf("stereo", geo.Mount(), geo.HW_PROFILE))
        hw_rgbd = ET.fromstring(render.render_sdf("rgbd", geo.Mount(), geo.HW_PROFILE))
        self.assertEqual(self._nested(hw, "OV9282_left", "width"), "1280")
        self.assertEqual(self._nested(hw, "OV9282_left", "height"), "800")
        self.assertEqual(self._nested(full, "OV9282_left", "fx"), self._nested(hw, "OV9282_left", "fx"))
        self.assertEqual(self._nested(hw, "OV9282_left", "update_rate"), "15")
        self.assertEqual(self._nested(full, "OV9282_left", "update_rate"), "30")
        self.assertEqual(self._nested(hw, "BNO086", "update_rate"), "200")
        fx = float(self._nested(hw, "OV9282_right", "fx"))
        tx = float(self._nested(hw, "OV9282_right", "tx"))
        self.assertAlmostEqual(tx / fx, -0.075, places=6)
        self.assertEqual(self._nested(hw_rgbd, "IMX378", "width"), "640")
        self.assertEqual(self._nested(hw_rgbd, "IMX378", "height"), "480")
        self.assertEqual(self._nested(hw_rgbd, "rgb_aligned_depth", "width"), "640")
        self.assertEqual(self._nested(hw_rgbd, "IMX378", "update_rate"), "15")
        self.assertEqual(self._nested(hw_rgbd, "rgb_aligned_depth", "update_rate"), "15")

    def test_rendered_urdf_loads_as_a_yaml_string(self):
        """The URDF is text robot_state_publisher can take as a string parameter."""
        # launch_ros runs yaml.safe_load on robot_description. A colon-space
        # inside an XML comment is a mapping, and StatePublisher rejects it.
        for mount in self.mounts:
            urdf = render.render_urdf(mount)
            for comment in re.findall(r"<!--(.*?)-->", urdf, flags=re.DOTALL):
                self.assertNotIn(": ", comment)
            loaded = yaml.safe_load(urdf)
            self.assertIsInstance(loaded, str)

    def test_checked_in_models_match_default_mount(self):
        """The checked-in models match a full-profile render at the default mount."""
        mount = geo.Mount()
        self.assertEqual(render.STEREO_SDF.read_text(encoding="utf-8"), render.render_sdf("stereo", mount))
        self.assertEqual(render.RGBD_SDF.read_text(encoding="utf-8"), render.render_sdf("rgbd", mount))
        self.assertEqual(render.URDF_PATH.read_text(encoding="utf-8"), render.render_urdf(mount))

    def test_urdf_joints_reference_real_links(self):
        """Every URDF joint parent and child is a link in the same file."""
        urdf = ET.fromstring(render.render_urdf(geo.Mount(pitch_deg=17)))
        links = {node.attrib["name"] for node in findall(urdf, "link")}
        self.assertIn(geo.RGB_OPTICAL, links)
        for joint in findall(urdf, "joint"):
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            self.assertIn(parent, links)
            self.assertIn(child, links)

    def test_x500_patch_file_keeps_other_includes(self):
        """The x500 patch rewrites the two camera poses and leaves the other include."""
        mount = geo.Mount(x=0.15, y=0.0, z=0.2, pitch_deg=17)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "model.sdf"
            path.write_text(self.x500_sample(), encoding="utf-8")
            render.patch_x500_depth_file(path, mount)
            text = path.read_text(encoding="utf-8")
        self.assertIn("model://x500", text)
        self.assertEqual(text.count(mount.pose_text()), 2)

    def x500_sample(self) -> str:
        """Minimal x500_depth SDF with an OakD include and a camera joint."""
        return """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="x500_depth">
    <include merge="true">
      <uri>model://x500</uri>
    </include>
    <include merge="true">
      <uri>model://OakD-Lite</uri>
      <pose>.12 .03 .242 0 0 0</pose>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
    </joint>
  </model>
</sdf>
"""

    def _sensor(self, root, name: str):
        """The sensor element named ``name``."""
        for node in findall(root, "sensor"):
            if node.attrib.get("name") == name:
                return node
        raise KeyError(name)

    def _nested(self, root, sensor_name: str, tag: str) -> str:
        """Text of the first ``tag`` inside the named sensor."""
        sensor = self._sensor(root, sensor_name)
        matches = [node for node in sensor.iter() if local(node.tag) == tag and node.text]
        if not matches:
            raise KeyError(tag)
        return matches[0].text.strip()


class HealthAndEvoTest(unittest.TestCase):
    """Health-log gates, launch-script text, and the evo command shape."""

    def test_tracking_loss_duration_and_loop(self):
        """A loss records a duration, and a loop closure is counted once."""
        log = HEALTH.TrackingLog(min_inliers=20)
        stats_ok = {"Odometry/Inliers/": 40.0}
        stats_lost = {"Odometry/Inliers/": 3.0}
        log.update(1.0, "t1", 1, 0, 0, stats_ok)
        log.update(2.0, "t2", 2, 0, 0, stats_lost)
        log.update(2.4, "t3", 3, 0, 0, stats_lost)
        events = log.update(4.0, "t4", 4, 7, 0, stats_ok)
        names = [event["event"] for event in events]
        self.assertIn("tracking_lost_end", names)
        end = next(event for event in events if event["event"] == "tracking_lost_end")
        self.assertAlmostEqual(end["duration_s"], 2.0)
        self.assertEqual(end["loss_count"], 1)
        self.assertIn("loop_closure", names)
        self.assertEqual(log.loop_count, 1)
        # The same closure on the same node is not counted twice.
        log.update(4.1, "t5", 4, 7, 0, stats_ok)
        self.assertEqual(log.loop_count, 1)

    def test_missing_inliers_are_not_a_loss(self):
        """A stats message with no inlier key is not a tracking loss."""
        log = HEALTH.TrackingLog()
        events = log.update(1.0, "t", 1, 0, 0, {"Loop/Id/": 0.0})
        self.assertEqual(events[0]["tracking"], "ok")
        self.assertEqual(log.loss_count, 0)

    def test_evo_commands_do_not_free_scale(self):
        """The evo commands align in SE3 and do not correct scale."""
        ape = EVAL.ape_command("gt.tum", "est.tum")
        rpe = EVAL.rpe_command("gt.tum", "est.tum")
        self.assertTrue(EVAL.se3_alignment_only(ape))
        self.assertTrue(EVAL.se3_alignment_only(rpe))
        self.assertNotIn("-as", ape + rpe)
        self.assertNotIn("--correct_scale", ape + rpe)
        self.assertFalse(EVAL.se3_alignment_only(ape + ["-as"]))
        self.assertTrue(EVAL.passes(0.1, 0.02, 0.5, 0.1))
        self.assertFalse(EVAL.passes(0.6, 0.02, 0.5, 0.1))
        self.assertFalse(EVAL.passes(0.1, 0.2, 0.5, 0.1))

    def test_parse_evo_rmse_row(self):
        """The rmse row is the number the gate compares."""
        text = "max 1.0\nrmse 0.25\nstd 0.1\n"
        self.assertAlmostEqual(EVAL.parse_evo_rmse(text), 0.25)

    def test_stereo_launch_uses_exact_sync(self):
        """Both stereo launches use exact sync and do not set an approx interval."""
        root = Path(__file__).resolve().parents[1] / "startFiles"
        stereo = (root / "gz_start_rtabmap_stereo.sh").read_text(encoding="utf-8")
        wrapper = (root / "gz_start_rtabmap.sh").read_text(encoding="utf-8")
        stereo_branch = wrapper.split("elif", 1)[0]
        for text in (stereo, stereo_branch):
            self.assertIn("approx_sync:=false", text)
            self.assertIn("wait_imu_to_init:=true", text)
            self.assertNotIn("approx_sync_max_interval", text)
            self.assertNotIn("approx_sync:=true", text)

    def _profile_args(self, profile: str, camera: str, extra_env: str = "") -> dict:
        """Source ``rtabmap_profile.sh`` and return the three launch variables."""
        script = Path(__file__).resolve().parents[1] / "startFiles" / "rtabmap_profile.sh"
        command = (
            "set -euo pipefail; "
            "unset CAM_STEREO_RES CAM_STEREO_WIDTH CAM_STEREO_HEIGHT CAM_RATE_HZ "
            "CAM_COLOR_WIDTH CAM_COLOR_HEIGHT IMU_RATE_HZ; "
            f'source "{script}"; '
            f"{extra_env}VISION_PROFILE={profile} rtabmap_profile_args {camera}; "
            'printf "%s\\n%s\\n%s\\n" "$RTAB_CFG" "$RTAB_ARGS" "$RTAB_ODOM"'
        )
        completed = subprocess.run(["bash", "-c", command], check=True, text=True, capture_output=True)
        cfg, args, odom = completed.stdout.splitlines()
        return {"RTAB_CFG": cfg, "RTAB_ARGS": args, "RTAB_ODOM": odom, "stderr": completed.stderr}

    def test_unset_vision_profile_shell_is_cpu(self):
        """An unset VISION_PROFILE resolves to the cpu profile."""
        script = Path(__file__).resolve().parents[1] / "startFiles" / "rtabmap_profile.sh"
        command = (
            "set -euo pipefail; "
            "unset VISION_PROFILE CAM_STEREO_RES CAM_STEREO_WIDTH CAM_STEREO_HEIGHT CAM_RATE_HZ "
            "CAM_COLOR_WIDTH CAM_COLOR_HEIGHT IMU_RATE_HZ; "
            f'source "{script}"; '
            "rtabmap_profile_args stereo; "
            'printf "%s\\n" "$VISION_PROFILE"'
        )
        completed = subprocess.run(["bash", "-c", command], check=True, text=True, capture_output=True)
        self.assertEqual(completed.stdout.strip(), "cpu")
        self.assertIn("vision profile=cpu", completed.stderr)
        self.assertIn("stereo=320x200", completed.stderr)
        self.assertIn("camera_hz=10", completed.stderr)
        self.assertNotIn("no GPU", completed.stderr)

    def test_container_mount_layout_loads_profile_and_failure_stops(self):
        """Rtabmap bind-mounts startFiles apart from includes/gz.

        ../oakd_s2 from that mount does not exist. The module is the
        includes/gz copy. A missing module must not look like a launch.
        """
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            gz = root / "volume" / "includes" / "gz"
            shutil.copytree(REPO / "includes" / "gz" / "oakd_s2", gz / "oakd_s2")
            shutil.copytree(REPO / "includes" / "gz" / "startFiles", gz / "startFiles")
            start = root / "volume" / "startFiles"
            shutil.copytree(REPO / "includes" / "gz" / "startFiles", start)
            self.assertFalse((root / "volume" / "oakd_s2").exists())
            script = start / "rtabmap_profile.sh"
            command = (
                "set -euo pipefail; "
                f'export HOME="{root}"; '
                "unset OAKD_S2_DIR VISION_PROFILE CAM_STEREO_RES CAM_STEREO_WIDTH "
                "CAM_STEREO_HEIGHT CAM_RATE_HZ CAM_COLOR_WIDTH CAM_COLOR_HEIGHT IMU_RATE_HZ; "
                f'source "{script}"; '
                "rtabmap_profile_args stereo; "
                'printf "%s\\n%s\\n" "$RTAB_CFG" "$RTAB_ARGS"'
            )
            completed = subprocess.run(
                ["bash", "-c", command], check=True, text=True, capture_output=True
            )
            cfg, args = completed.stdout.splitlines()
            self.assertTrue(str(cfg).endswith("startFiles/rtabmap_profiles/cpu.ini"))
            self.assertTrue(Path(cfg).is_file())
            self.assertIn("/volume/includes/gz/startFiles/rtabmap_profiles/cpu.ini", cfg)
            self.assertTrue(args.startswith("-d"))
            self.assertIn("vision profile=cpu", completed.stderr)
            self.assertIn("stereo=320x200", completed.stderr)
            self.assertIn("camera_hz=10", completed.stderr)

            broken = root / "empty"
            broken.mkdir()
            failed = subprocess.run(
                [
                    "bash",
                    "-c",
                    "set -u; "
                    f'export HOME="{broken}"; '
                    "unset OAKD_S2_DIR VISION_PROFILE; "
                    f'source "{script}"; '
                    "rtabmap_profile_args stereo || exit 1; "
                    "echo launched",
                ],
                text=True,
                capture_output=True,
            )
            self.assertNotEqual(failed.returncode, 0)
            self.assertNotIn("launched", failed.stdout)
            self.assertIn("missing rtabmap_params.py", failed.stderr)
        start_dir = Path(__file__).resolve().parents[1] / "startFiles"
        for name in ("gz_start_rtabmap.sh", "gz_start_rtabmap_stereo.sh", "gz_start_rtabmap_rgbd.sh"):
            text = (start_dir / name).read_text(encoding="utf-8")
            self.assertIn("refusing to launch RTAB-Map without a profile", text)
            self.assertIn("exit 1", text)

    def test_launch_scripts_use_real_parameter_names(self):
        """The launches pass real RTAB-Map keys and the profile ini."""
        root = Path(__file__).resolve().parents[1] / "startFiles"
        names = (
            "gz_start_rtabmap.sh",
            "gz_start_rtabmap_stereo.sh",
            "gz_start_rtabmap_rgbd.sh",
            "rtabmap_profile.sh",
        )
        texts = {name: (root / name).read_text(encoding="utf-8") for name in names}
        for text in texts.values():
            self.assertNotIn("--MaxFeatures", text)
            self.assertNotIn("--NormalsSegmentation", text)
            self.assertNotIn("rtabmapviz:=", text)
            self.assertNotIn("MaxFeatures:=", text)
        profile = texts["rtabmap_profile.sh"]
        self.assertIn("rtabmap_params.py", profile)
        self.assertNotIn("--Vis/MaxFeatures", profile)
        self.assertNotIn("--Grid/NormalsSegmentation", profile)
        full_ini = rtab.load_profile_ini("full")[1]
        self.assertEqual(full_ini["Odom/ResetCountdown"], "1")
        self.assertEqual(full_ini["Grid/NormalsSegmentation"], "false")
        self.assertEqual(full_ini["Vis/MaxFeatures"], "1000")
        for name in ("gz_start_rtabmap.sh", "gz_start_rtabmap_stereo.sh", "gz_start_rtabmap_rgbd.sh"):
            self.assertIn("rtabmap_profile.sh", texts[name])
            self.assertIn("rtabmap_viz:=", texts[name])
            self.assertIn('cfg:="${RTAB_CFG}"', texts[name])
        wrapper = texts["gz_start_rtabmap.sh"]
        stereo_branch, rgbd_branch = wrapper.split("elif", 1)
        self.assertIn("approx_sync:=false", stereo_branch)
        self.assertIn("rtabmap_profile_args stereo", stereo_branch)
        self.assertIn("rtabmap_profile_args rgbd-wrapper", rgbd_branch)
        full_wrapper = self._profile_args("full", "rgbd-wrapper")
        self.assertIn("--Grid/3D true", full_wrapper["RTAB_ARGS"])
        self.assertIn("--Grid/RayTracing true", full_wrapper["RTAB_ARGS"])
        self.assertIn("--Vis/DepthAsMask true", full_wrapper["RTAB_ODOM"])
        self.assertIn("approx_sync:=true", rgbd_branch)
        self.assertIn("wait_imu_to_init:=true", rgbd_branch)
        rgbd = texts["gz_start_rtabmap_rgbd.sh"]
        self.assertIn("approx_sync:=true", rgbd)
        self.assertIn("rtabmap_profile_args rgbd", rgbd)
        self.assertIn("wait_imu_to_init:=true", rgbd)
        full_rgbd = self._profile_args("full", "rgbd")
        self.assertIn("--Vis/DepthAsMask true", full_rgbd["RTAB_ODOM"])
        self.assertNotIn("--Grid/3D", full_rgbd["RTAB_ARGS"])

    def _feed_odom(self, log, rows, dt=0.1):
        """rows are synthetic OdomInfo samples: (lost, features, inliers).

        Default spacing is 10 Hz of sim time, above VISION_MIN_ODOM_HZ.
        """
        for index, (lost, features, inliers) in enumerate(rows):
            log.observe_odom(index * dt, f"t{index}", lost, features, inliers)

    def _without_vision_env(self):
        """Drop the vision gate variables so the profile defaults are visible."""
        keys = (
            "VISION_MAX_LOST_STREAK",
            "VISION_MAX_RECOVERY_FRAMES",
            "VISION_MIN_MEDIAN_FEATURES",
            "VISION_MIN_INLIERS",
            "VISION_MIN_ODOM_HZ",
            "VISION_PROFILE",
        )
        return {key: os.environ.get(key) for key in keys}

    def _restore_env(self, saved):
        """Put back the environment ``_without_vision_env`` removed."""
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def test_env_files_define_vision_gates(self):
        """The env files name the vision gates the health log reads."""
        for name in (".env", ".env.example"):
            text = (REPO / name).read_text(encoding="utf-8")
            self.assertRegex(text, r"(?m)^VISION_PROFILE=cpu$")
            self.assertNotRegex(text, r"(?m)^VISION_PROFILE=full$")
            self.assertIn("VISION_MAX_LOST_STREAK=3", text)
            self.assertIn("VISION_MAX_RECOVERY_FRAMES=2", text)
            self.assertIn("VISION_MIN_ODOM_HZ=7", text)
            self.assertIn("VISION_MIN_MEDIAN_FEATURES=120", text)
            self.assertIn("VISION_MIN_MEDIAN_FEATURES=40", text)
            self.assertIn("VISION_MIN_INLIERS=20", text)
            self.assertIn("VISION_MIN_INLIERS=15", text)
            self.assertIn("VISION_MIN_MEDIAN_FEATURES=80", text)
            self.assertIn("not yet measured", text)
        compose = (REPO / "docker-compose-px4.yml").read_text(encoding="utf-8")
        self.assertEqual(compose.count("${VISION_PROFILE:-cpu}"), 3)
        self.assertNotIn("${VISION_PROFILE:-full}", compose)
        up = (REPO / "scripts" / "up.sh").read_text(encoding="utf-8")
        self.assertIn("_restore_or_default VISION_PROFILE cpu", up)
        shell = (REPO / "includes" / "gz" / "startFiles" / "rtabmap_profile.sh").read_text(encoding="utf-8")
        self.assertIn('VISION_PROFILE="${VISION_PROFILE:-cpu}"', shell)

    def test_thresholds_from_env_and_file(self):
        """An env file fills a missing gate and the process environment wins."""
        saved = self._without_vision_env()
        try:
            for key in saved:
                os.environ.pop(key, None)
            got = HEALTH.thresholds_from_env()
            self.assertEqual(HEALTH.profile_name_from_env(), "cpu")
            self.assertEqual(got.max_lost_streak, 3)
            self.assertEqual(got.max_recovery_frames, 2)
            self.assertEqual(got.min_median_features, 40)
            self.assertEqual(got.min_inliers, 15)
            HEALTH.load_repo_env()
            filled = HEALTH.thresholds_from_env()
            self.assertEqual(filled.max_lost_streak, 3)
            self.assertEqual(filled.min_median_features, 40)
            self.assertEqual(filled.min_inliers, 15)
            os.environ["VISION_MAX_LOST_STREAK"] = "9"
            os.environ["VISION_MAX_RECOVERY_FRAMES"] = "1"
            os.environ["VISION_MIN_MEDIAN_FEATURES"] = "100"
            os.environ["VISION_MIN_INLIERS"] = "15"
            HEALTH.load_repo_env()
            override = HEALTH.thresholds_from_env()
            self.assertEqual(override.max_lost_streak, 9)
            self.assertEqual(override.max_recovery_frames, 1)
            self.assertEqual(override.min_median_features, 100)
            self.assertEqual(override.min_inliers, 15)
        finally:
            self._restore_env(saved)

    def test_odom_info_sequence_within_gates(self):
        """A healthy odom_info sequence passes every gate."""
        log = HEALTH.TrackingLog()
        # One lost frame, then tracking returns on the next frame.
        rows = [(False, 800, 50)] * 4 + [(True, 12, 0), (False, 700, 40)]
        self._feed_odom(log, rows)
        summary = log.summary("end")
        self.assertEqual(summary["metrics"]["lost_streak"]["value"], 1)
        self.assertEqual(summary["metrics"]["lost_streak"]["threshold"], 3)
        self.assertTrue(summary["metrics"]["lost_streak"]["pass"])
        self.assertEqual(summary["metrics"]["recovery_frames"]["value"], 1)
        self.assertEqual(summary["metrics"]["recovery_frames"]["episodes"], [1])
        self.assertEqual(summary["metrics"]["recovery_frames"]["open_frames"], 0)
        self.assertTrue(summary["metrics"]["recovery_frames"]["pass"])
        self.assertGreaterEqual(summary["metrics"]["median_features"]["value"], 120)
        self.assertEqual(summary["metrics"]["median_features"]["threshold"], 120)
        self.assertTrue(summary["metrics"]["median_features"]["pass"])
        self.assertTrue(summary["metrics"]["inliers"]["pass"])
        self.assertEqual(summary["metrics"]["inliers"]["value"]["min"], 0)
        self.assertGreaterEqual(summary["metrics"]["inliers"]["value"]["below_threshold"], 1)
        self.assertTrue(summary["pass"])
        for name in HEALTH.METRIC_NAMES:
            metric = summary["metrics"][name]
            self.assertIn("value", metric)
            self.assertIn("threshold", metric)
            self.assertIsInstance(metric["pass"], bool)

    def test_two_recoveries_use_the_worst_episode(self):
        """Two recoveries are scored by the longer episode."""
        log = HEALTH.TrackingLog()
        rows = [
            (False, 600, 30),
            (True, 4, 0),
            (False, 600, 30),
            (True, 4, 0),
            (True, 4, 0),
            (False, 600, 30),
        ]
        self._feed_odom(log, rows)
        recovery = log.summary("end")["metrics"]["recovery_frames"]
        self.assertEqual(recovery["episodes"], [1, 2])
        self.assertEqual(recovery["value"], 2)
        self.assertTrue(recovery["pass"])

    def test_three_lost_frames_fail_recovery_and_keep_the_streak(self):
        """Three lost frames fail recovery and still pass the streak cap."""
        log = HEALTH.TrackingLog()
        rows = [(False, 600, 30), (True, 5, 0), (True, 5, 0), (True, 5, 0), (False, 600, 30)]
        self._feed_odom(log, rows)
        summary = log.summary("end")
        self.assertEqual(summary["metrics"]["lost_streak"]["value"], 3)
        self.assertTrue(summary["metrics"]["lost_streak"]["pass"])
        self.assertEqual(summary["metrics"]["recovery_frames"]["value"], 3)
        self.assertFalse(summary["metrics"]["recovery_frames"]["pass"])
        self.assertFalse(summary["pass"])

    def test_four_lost_frames_fail_the_streak(self):
        """Four lost frames fail the streak gate."""
        log = HEALTH.TrackingLog()
        rows = [(False, 600, 30)] + [(True, 5, 0)] * 4 + [(False, 600, 30)]
        self._feed_odom(log, rows)
        summary = log.summary("end")
        self.assertEqual(summary["metrics"]["lost_streak"]["value"], 4)
        self.assertFalse(summary["metrics"]["lost_streak"]["pass"])
        self.assertFalse(summary["pass"])

    def test_open_loss_fails_recovery_after_finish(self):
        """A loss that is still open when the log ends fails recovery."""
        log = HEALTH.TrackingLog()
        self._feed_odom(log, [(False, 600, 30), (True, 5, 0)])
        end = log.finish("end")
        self.assertEqual(end[0]["event"], "tracking_lost_end")
        self.assertTrue(end[0]["open"])
        recovery = log.summary("end")["metrics"]["recovery_frames"]
        self.assertEqual(recovery["open_frames"], 1)
        self.assertFalse(recovery["pass"])
        self.assertFalse(log.summary("end")["pass"])

    def test_low_median_features_fail(self):
        """A feature median under the floor fails that gate."""
        log = HEALTH.TrackingLog()
        self._feed_odom(log, [(False, 80, 40), (False, 100, 40)])
        features = log.summary("end")["metrics"]["median_features"]
        self.assertEqual(features["value"], 90)
        self.assertEqual(features["threshold"], 120)
        self.assertFalse(features["pass"])
        self.assertTrue(log.summary("end")["metrics"]["inliers"]["pass"])

    def test_even_feature_count_median_meets_the_floor(self):
        """An even feature count uses the average of the two middle values."""
        log = HEALTH.TrackingLog()
        self._feed_odom(log, [(False, 100, 40), (False, 140, 40)])
        features = log.summary("end")["metrics"]["median_features"]
        self.assertEqual(features["value"], 120)
        self.assertTrue(features["pass"])

    def test_tracked_frame_below_inlier_floor_fails(self):
        """A tracked frame under the inlier floor fails that gate."""
        log = HEALTH.TrackingLog()
        self._feed_odom(log, [(False, 600, 10), (False, 600, 40)])
        inliers = log.summary("end")["metrics"]["inliers"]
        self.assertFalse(inliers["pass"])
        self.assertEqual(inliers["threshold"], 20)
        self.assertFalse(log.summary("end")["pass"])

    def test_reset_frame_zero_inliers_do_not_fail_the_inlier_gate(self):
        """The zero-inlier frame after a reset is left out of the inlier gate."""
        log = HEALTH.TrackingLog()
        # Lost frame, then the reset frame (not lost, 0 inliers), then tracking.
        self._feed_odom(log, [(False, 600, 40), (True, 8, 0), (False, 500, 0), (False, 600, 40)])
        summary = log.summary("end")
        self.assertEqual(summary["metrics"]["lost_streak"]["value"], 1)
        self.assertTrue(summary["metrics"]["recovery_frames"]["pass"])
        self.assertTrue(summary["metrics"]["inliers"]["pass"])
        self.assertEqual(summary["metrics"]["inliers"]["value"]["min"], 0)
        self.assertGreaterEqual(summary["metrics"]["inliers"]["value"]["below_threshold"], 1)
        self.assertTrue(summary["pass"])
        # A 0-inlier frame that did not follow a loss still fails the gate.
        other = HEALTH.TrackingLog()
        self._feed_odom(other, [(False, 600, 40), (False, 600, 0)])
        self.assertFalse(other.summary("end")["metrics"]["inliers"]["pass"])

    def test_lost_frame_inliers_do_not_fail_the_inlier_gate(self):
        """Inliers on a lost frame do not fail the inlier gate."""
        log = HEALTH.TrackingLog()
        self._feed_odom(log, [(False, 600, 40), (True, 1, 0), (False, 600, 40)])
        summary = log.summary("end")
        self.assertTrue(summary["metrics"]["inliers"]["pass"])
        self.assertEqual(summary["metrics"]["inliers"]["value"]["min"], 0)
        self.assertEqual(summary["metrics"]["inliers"]["value"]["below_threshold"], 1)
        self.assertTrue(summary["pass"])

    def test_empty_log_fails_features_and_inliers(self):
        """An empty log fails the feature and inlier gates."""
        summary = HEALTH.TrackingLog().summary("end")
        self.assertFalse(summary["metrics"]["median_features"]["pass"])
        self.assertIsNone(summary["metrics"]["median_features"]["value"])
        self.assertFalse(summary["metrics"]["inliers"]["pass"])
        self.assertFalse(summary["pass"])

    def test_odom_info_is_preferred_over_later_info_stats(self):
        """Later info stats do not add samples once odom info has been seen."""
        log = HEALTH.TrackingLog()
        self._feed_odom(log, [(False, 600, 40)] * 3)
        events = log.update(10.0, "wall", 9, 4, 0, {"Odometry/Inliers/": 1.0})
        self.assertEqual([event["event"] for event in events], ["loop_closure"])
        self.assertEqual(log.loss_count, 0)
        self.assertEqual(log.loop_count, 1)
        summary = log.summary("end")
        self.assertEqual(summary["metrics"]["lost_streak"]["value"], 0)
        self.assertTrue(summary["pass"])
        log.update(11.0, "wall", 9, 4, 0, {"Odometry/Inliers/": 1.0})
        self.assertEqual(log.loop_count, 1)

    def test_cpu_profile_parameters_only_when_selected(self):
        """The cpu disparity cap is applied only when that profile is selected."""
        # The cpu stereo set is the previous command line, including VisKeyFrameThr 30.
        cpu_stereo = {
            "Optimizer/GravitySigma": "0.1",
            "Vis/FeatureType": "10",
            "Kp/DetectorStrategy": "10",
            "Vis/MaxFeatures": "400",
            "Vis/MinInliers": "15",
            "Kp/MaxFeatures": "300",
            "Grid/MapFrameProjection": "true",
            "Grid/NormalsSegmentation": "false",
            "Grid/MaxGroundHeight": "1.0",
            "Grid/MaxObstacleHeight": "2.0",
            "Grid/CellSize": "0.1",
            "Grid/RangeMax": "5",
            "Rtabmap/DetectionRate": "1",
            "RGBD/StartAtOrigin": "true",
            "Stereo/MaxDisparity": "64",
            "Odom/Strategy": "0",
            "Odom/ResetCountdown": "1",
            "Odom/VisKeyFrameThr": "30",
            "OdomF2M/MaxSize": "1000",
            "Vis/CorGuessWinSize": "20",
            "Vis/EstimationType": "1",
        }
        self.assertEqual(rtab.effective_parameters("cpu", "stereo"), cpu_stereo)
        cpu_ini = rtab.load_profile_ini("cpu")[1]
        full_ini = rtab.load_profile_ini("full")[1]
        hw_ini = rtab.load_profile_ini("hw")[1]
        self.assertEqual(cpu_ini["Odom/VisKeyFrameThr"], "30")
        self.assertNotIn("Odom/VisKeyFrameThr", full_ini)
        self.assertEqual(hw_ini["Odom/VisKeyFrameThr"], "40")
        self.assertEqual(hw_ini["Vis/MaxFeatures"], "600")
        self.assertNotIn("Grid/CellSize", hw_ini)
        self.assertNotIn("Stereo/MaxDisparity", hw_ini)
        self.assertNotIn("Rtabmap/DetectionRate", full_ini)
        for name in ("cpu", "full", "hw"):
            for camera in ("stereo", "rgbd", "rgbd-wrapper"):
                launched = self._profile_args(name, camera)
                self.assertTrue(launched["RTAB_CFG"].endswith(f"/{name}.ini"), launched["RTAB_CFG"])
                self.assertIn(f"vision profile={name} camera={camera}", launched["stderr"])
                self.assertIn("stereo=", launched["stderr"])
                logged = {}
                for line in launched["stderr"].splitlines():
                    if line.startswith("rtabmap_param source=ini "):
                        key, value = line[len("rtabmap_param source=ini ") :].split("=", 1)
                        logged[key] = value
                self.assertEqual(logged, rtab.load_profile_ini(name)[1])
                self.assertNotIn("--Vis/MaxFeatures", launched["RTAB_ARGS"])
                self.assertNotIn("VisKeyFrameThr", launched["RTAB_ARGS"] + launched["RTAB_ODOM"])
        cpu = self._profile_args("cpu", "stereo")
        full = self._profile_args("full", "stereo")
        self.assertIn("--Stereo/MaxDisparity 64", cpu["RTAB_ARGS"])
        self.assertNotIn("MaxDisparity", full["RTAB_ARGS"])
        self.assertNotIn("MaxDisparity", self._profile_args("cpu", "rgbd")["RTAB_ARGS"])
        self.assertNotIn("MaxDisparity", self._profile_args("hw", "rgbd")["RTAB_ARGS"])
        self.assertIn("--Vis/DepthAsMask true", self._profile_args("cpu", "rgbd")["RTAB_ODOM"])
        self.assertIn("--Vis/DepthAsMask true", self._profile_args("hw", "rgbd")["RTAB_ODOM"])
        self.assertNotIn("DepthAsMask", cpu["RTAB_ODOM"])
        wrapper = self._profile_args("cpu", "rgbd-wrapper")
        self.assertIn("--Grid/3D true", wrapper["RTAB_ARGS"])
        self.assertNotIn("MaxDisparity", wrapper["RTAB_ARGS"])
        self.assertIn("--Grid/3D true", self._profile_args("full", "rgbd-wrapper")["RTAB_ARGS"])
        self.assertIn("--Grid/3D true", self._profile_args("hw", "rgbd-wrapper")["RTAB_ARGS"])
        low = self._profile_args("full", "rgbd", extra_env="CAM_STEREO_RES=640x400 ")
        self.assertIn("below 1280x720", low["stderr"])
        self.assertIn("stereo=640x400", low["stderr"])
        self.assertNotIn("below 1280x720", full["stderr"])
        self.assertNotIn("below 1280x720", cpu["stderr"])
        self.assertIn("below 1280x720", self._profile_args("hw", "stereo", extra_env="CAM_STEREO_RES=320x200 ")["stderr"])
        for name in ("gz_start_rtabmap.sh", "gz_start_rtabmap_stereo.sh", "gz_start_rtabmap_rgbd.sh"):
            text = (Path(__file__).resolve().parents[1] / "startFiles" / name).read_text(encoding="utf-8")
            self.assertIn("source ", text)
            self.assertIn("rtabmap_profile.sh", text)

    def test_sim_time_odom_hz_is_gated_and_wall_rate_is_reported(self):
        """The odom gate uses sim time, and the wall rate is only reported."""
        log = HEALTH.TrackingLog()
        for index in range(9):
            log.observe_odom(index / 8.0, f"t{index}", False, 600, 40, wall_s=index / 2.0)
        metric = log.summary("end")["metrics"]["odom_hz"]
        self.assertAlmostEqual(metric["value"], 8.0, places=5)
        self.assertAlmostEqual(metric["wall_hz"], 2.0, places=5)
        self.assertAlmostEqual(metric["ratio"], 0.25, places=5)
        self.assertEqual(metric["threshold"], 7)
        self.assertTrue(metric["pass"])

    def test_slow_sim_odom_fails_hz_even_if_wall_is_fast(self):
        """Slow sim-time odometry fails the rate gate even when the wall rate is fast."""
        log = HEALTH.TrackingLog()
        for index in range(5):
            log.observe_odom(float(index), "t", False, 600, 40, wall_s=index * 0.1)
        metric = log.summary("end")["metrics"]["odom_hz"]
        self.assertEqual(metric["value"], 1)
        self.assertAlmostEqual(metric["wall_hz"], 10.0, places=5)
        self.assertFalse(metric["pass"])
        self.assertFalse(log.summary("end")["pass"])

    def test_cpu_profile_lowers_feature_and_inlier_defaults(self):
        """The cpu profile lowers the feature and inlier floors."""
        saved = self._without_vision_env()
        try:
            for key in saved:
                os.environ.pop(key, None)
            os.environ["VISION_PROFILE"] = "cpu"
            got = HEALTH.thresholds_from_env()
            self.assertEqual(got.min_median_features, 40)
            self.assertEqual(got.min_inliers, 15)
            self.assertEqual(got.min_odom_hz, 7)
            os.environ["VISION_PROFILE"] = "hw"
            os.environ.pop("VISION_MIN_MEDIAN_FEATURES", None)
            os.environ.pop("VISION_MIN_INLIERS", None)
            hw = HEALTH.thresholds_from_env()
            self.assertEqual(hw.min_median_features, 80)
            self.assertEqual(hw.min_inliers, 20)
            os.environ["VISION_PROFILE"] = "nope"
            with self.assertRaises(ValueError):
                HEALTH.thresholds_from_env()
            os.environ["VISION_PROFILE"] = "cpu"
            os.environ["VISION_MIN_MEDIAN_FEATURES"] = "500"
            os.environ["VISION_MIN_INLIERS"] = "20"
            explicit = HEALTH.thresholds_from_env()
            self.assertEqual(explicit.min_median_features, 500)
            self.assertEqual(explicit.min_inliers, 20)
        finally:
            self._restore_env(saved)

    def test_min_inliers_flag_does_not_reset_other_thresholds(self):
        """The min-inliers flag changes only that threshold."""
        saved = self._without_vision_env()
        try:
            for key in saved:
                os.environ.pop(key, None)
            os.environ["VISION_MIN_ODOM_HZ"] = "9"
            os.environ["VISION_MIN_MEDIAN_FEATURES"] = "80"
            os.environ["VISION_MAX_LOST_STREAK"] = "4"
            os.environ["VISION_PROFILE"] = "cpu"
            got, args = HEALTH.thresholds_from_args(["--min-inliers", "11"])
            self.assertEqual(args.min_inliers, 11)
            self.assertEqual(got.min_inliers, 11)
            self.assertEqual(got.min_odom_hz, 9)
            self.assertEqual(got.min_median_features, 80)
            self.assertEqual(got.max_lost_streak, 4)
            self.assertEqual(got.max_recovery_frames, 2)
            untouched, _ = HEALTH.thresholds_from_args([])
            self.assertEqual(untouched.min_inliers, 15)
            self.assertEqual(untouched.min_odom_hz, 9)
            self.assertEqual(untouched.min_median_features, 80)
        finally:
            self._restore_env(saved)

    def test_rate_probe_summary(self):
        """The rate probe summary reports each topic and the real-time factor."""
        result = PROBE.summarize(
            {
                "/imu": 1000,
                "/rtabmap/odom": 80,
                "/camera/stereo/left/image_raw": 100,
            },
            window_s=10,
            clock_span=4,
        )
        self.assertAlmostEqual(result["rates_hz"]["/imu"], 100.0)
        self.assertAlmostEqual(result["rates_hz"]["/rtabmap/odom"], 8.0)
        self.assertAlmostEqual(result["rates_hz"]["/camera/stereo/left/image_raw"], 10.0)
        self.assertAlmostEqual(result["rates_hz"]["/camera/rgb/image_raw"], 0.0)
        self.assertAlmostEqual(result["rtf"], 0.4)
        line = PROBE.format_summary(result)
        self.assertIn("odom 8.00 Hz", line)
        self.assertIn("rtf 0.400", line)
        self.assertIn("over 10.0s", line)
        events = PROBE.jsonl_events(result)
        self.assertEqual(events[-1]["event"], "summary")
        self.assertEqual(events[-1]["rtf"], 0.4)


if __name__ == "__main__":
    unittest.main()
