#!/usr/bin/env python3
"""Pose, baseline, and SLAM-gate checks that do not need Gazebo or ROS."""

from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import geometry as geo  # noqa: E402
import render_oakd as render  # noqa: E402


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPO = HERE.parents[2]
HEALTH = load_module(REPO / "HealthCheck" / "rtabmap_health_log.py", "rtabmap_health_log")
EVAL = load_module(REPO / "scripts" / "eval_slam_accuracy.py", "eval_slam_accuracy")


def pose_values(text: str):
    return [float(item) for item in text.split()]


def local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def findall(root, name: str):
    return [node for node in root.iter() if local(node.tag) == name]


def sensor_pose(root, sensor_name: str):
    for node in findall(root, "sensor"):
        if node.attrib.get("name") == sensor_name:
            pose = next(child for child in node if local(child.tag) == "pose")
            return pose_values(pose.text)
    raise KeyError(sensor_name)


def child_text(node, name: str) -> str:
    for child in node:
        if local(child.tag) == name and child.text:
            return child.text.strip()
    return ""


class GeometryTest(unittest.TestCase):
    def test_stereo_tx_uses_the_shared_left_fx(self):
        shared = geo.stereo_tx(geo.LEFT_INTRINSICS["fx"])
        self.assertAlmostEqual(shared, -geo.LEFT_INTRINSICS["fx"] * 0.075, places=9)
        self.assertLess(shared, 0.0)
        # The unused right calibration would have produced a different Tx.
        self.assertNotAlmostEqual(shared, geo.stereo_tx(geo.RIGHT_INTRINSICS["fx"]), places=3)
        left_p = geo.rectified_p(right=False)
        right_p = geo.rectified_p(right=True)
        self.assertEqual(geo.rectified_k()[0], left_p[0])
        self.assertEqual(left_p[0], right_p[0])
        self.assertEqual(left_p[2], right_p[2])
        self.assertEqual(left_p[5], right_p[5])
        self.assertEqual(left_p[6], right_p[6])
        self.assertEqual(left_p[3], 0.0)
        self.assertAlmostEqual(right_p[3], shared, places=9)

    def test_corrected_projection_fills_zero_tx_once(self):
        fx = geo.RIGHT_INTRINSICS["fx"]
        k = [fx, 0.0, 10.0, 0.0, fx, 20.0, 0.0, 0.0, 1.0]
        zeros = [0.0] * 12
        filled = geo.corrected_projection(zeros, k)
        self.assertAlmostEqual(filled[0], fx)
        self.assertAlmostEqual(filled[3], geo.stereo_tx(fx))
        again = geo.corrected_projection(filled, k)
        self.assertEqual(again[3], filled[3])

    def test_corrected_projection_rejects_missing_fx(self):
        self.assertIsNone(geo.corrected_projection([0.0] * 12, [0.0] * 9))

    def test_color_fov_matches_af_datasheet(self):
        color = geo.color_intrinsics()
        hfov = math.degrees(geo.hfov_from_intrinsics(color))
        vfov = math.degrees(geo.vfov_from_intrinsics(color))
        self.assertAlmostEqual(hfov, 66.0, places=6)
        self.assertAlmostEqual(color["fx"], color["fy"], places=6)
        self.assertAlmostEqual(vfov, 52.0, delta=0.15)
        self.assertAlmostEqual(geo.diagonal_fov_deg(hfov, vfov), 78.0, delta=0.2)

    def test_calibration_hfov_is_not_silently_replaced(self):
        hfov = math.degrees(geo.hfov_from_intrinsics(geo.LEFT_INTRINSICS))
        self.assertAlmostEqual(hfov, 76.4, delta=0.1)
        self.assertLess(hfov, 80.0)

    def test_optical_axes_and_baseline(self):
        rotation = geo.optical_rotation()
        self.assertAlmostEqual(geo.matvec(rotation, (0.0, 0.0, 1.0))[0], 1.0, places=6)
        baseline = geo.right_in_left_optical()
        self.assertAlmostEqual(baseline[0], geo.BASELINE_M, places=6)
        self.assertAlmostEqual(baseline[1], 0.0, places=6)
        self.assertAlmostEqual(baseline[2], 0.0, places=6)

    def test_positive_pitch_points_down(self):
        forward = geo.matvec(geo.rot_y(math.radians(17.0)), (1.0, 0.0, 0.0))
        self.assertLess(forward[2], 0.0)


class RenderTest(unittest.TestCase):
    def setUp(self):
        self.mounts = [
            geo.Mount(pitch_deg=0.0),
            geo.Mount(),
            geo.Mount(pitch_deg=30.0),
            geo.Mount(x=0.2, y=-0.01, z=0.3, pitch_deg=-10.0),
        ]

    def test_sensor_poses_match_across_pitches(self):
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
        stereo = ET.fromstring(render.render_sdf("stereo", geo.Mount()))
        links = {node.attrib["name"] for node in findall(ET.fromstring(render.render_urdf(geo.Mount())), "link")}
        for sensor_name, frame in (
            ("OV9282_left", geo.STEREO_LEFT_OPTICAL),
            ("OV9282_right", geo.STEREO_RIGHT_OPTICAL),
            ("BNO086", geo.IMU_FRAME),
        ):
            sensor = self._sensor(stereo, sensor_name)
            self.assertIn(child_text(sensor, "gz_frame_id"), links)
        right_tx = float(self._nested(stereo, "OV9282_right", "tx"))
        self.assertAlmostEqual(right_tx, geo.stereo_tx(geo.LEFT_INTRINSICS["fx"]), places=4)
        self.assertAlmostEqual(float(self._nested(stereo, "OV9282_left", "tx")), 0.0, places=6)
        for field in ("fx", "fy", "cx", "cy"):
            self.assertAlmostEqual(
                float(self._nested(stereo, "OV9282_left", field)),
                float(self._nested(stereo, "OV9282_right", field)),
                places=6,
            )
            self.assertAlmostEqual(
                float(self._nested(stereo, "OV9282_left", field)),
                geo.LEFT_INTRINSICS[field],
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

    def test_checked_in_models_match_default_mount(self):
        mount = geo.Mount()
        self.assertEqual(render.STEREO_SDF.read_text(encoding="utf-8"), render.render_sdf("stereo", mount))
        self.assertEqual(render.RGBD_SDF.read_text(encoding="utf-8"), render.render_sdf("rgbd", mount))
        self.assertEqual(render.URDF_PATH.read_text(encoding="utf-8"), render.render_urdf(mount))

    def test_urdf_joints_reference_real_links(self):
        urdf = ET.fromstring(render.render_urdf(geo.Mount(pitch_deg=17)))
        links = {node.attrib["name"] for node in findall(urdf, "link")}
        self.assertIn(geo.RGB_OPTICAL, links)
        for joint in findall(urdf, "joint"):
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            self.assertIn(parent, links)
            self.assertIn(child, links)

    def test_x500_patch_file_keeps_other_includes(self):
        mount = geo.Mount(x=0.15, y=0.0, z=0.2, pitch_deg=17)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "model.sdf"
            path.write_text(self.x500_sample(), encoding="utf-8")
            render.patch_x500_depth_file(path, mount)
            text = path.read_text(encoding="utf-8")
        self.assertIn("model://x500", text)
        self.assertEqual(text.count(mount.pose_text()), 2)

    def x500_sample(self) -> str:
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
        for node in findall(root, "sensor"):
            if node.attrib.get("name") == name:
                return node
        raise KeyError(name)

    def _nested(self, root, sensor_name: str, tag: str) -> str:
        sensor = self._sensor(root, sensor_name)
        matches = [node for node in sensor.iter() if local(node.tag) == tag and node.text]
        if not matches:
            raise KeyError(tag)
        return matches[0].text.strip()


class HealthAndEvoTest(unittest.TestCase):
    def test_tracking_loss_duration_and_loop(self):
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
        log = HEALTH.TrackingLog()
        events = log.update(1.0, "t", 1, 0, 0, {"Loop/Id/": 0.0})
        self.assertEqual(events[0]["tracking"], "ok")
        self.assertEqual(log.loss_count, 0)

    def test_evo_commands_do_not_free_scale(self):
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
        text = "max 1.0\nrmse 0.25\nstd 0.1\n"
        self.assertAlmostEqual(EVAL.parse_evo_rmse(text), 0.25)

    def test_stereo_launch_uses_exact_sync(self):
        root = Path(__file__).resolve().parents[1] / "startFiles"
        stereo = (root / "gz_start_rtabmap_stereo.sh").read_text(encoding="utf-8")
        wrapper = (root / "gz_start_rtabmap.sh").read_text(encoding="utf-8")
        stereo_branch = wrapper.split("elif", 1)[0]
        for text in (stereo, stereo_branch):
            self.assertIn("approx_sync:=false", text)
            self.assertIn("wait_imu_to_init:=true", text)
            self.assertNotIn("approx_sync_max_interval", text)
            self.assertNotIn("approx_sync:=true", text)

    def test_launch_scripts_use_real_parameter_names(self):
        root = Path(__file__).resolve().parents[1] / "startFiles"
        names = (
            "gz_start_rtabmap.sh",
            "gz_start_rtabmap_stereo.sh",
            "gz_start_rtabmap_rgbd.sh",
        )
        texts = {name: (root / name).read_text(encoding="utf-8") for name in names}
        for text in texts.values():
            self.assertNotIn("--MaxFeatures", text)
            self.assertNotIn("--NormalsSegmentation", text)
            self.assertNotIn("rtabmapviz:=", text)
            self.assertNotIn("MaxFeatures:=", text)
            self.assertIn("Odom/ResetCountdown 1", text)
            self.assertIn("rtabmap_viz:=", text)
            self.assertIn("--Grid/NormalsSegmentation false", text)
            self.assertIn("--Vis/MaxFeatures 1000", text)
        wrapper = texts["gz_start_rtabmap.sh"]
        stereo_branch, rgbd_branch = wrapper.split("elif", 1)
        self.assertIn("approx_sync:=false", stereo_branch)
        self.assertIn("--Grid/3D true", rgbd_branch)
        self.assertIn("--Grid/RayTracing true", rgbd_branch)
        self.assertIn("--Vis/DepthAsMask true", rgbd_branch)
        self.assertIn("approx_sync:=true", rgbd_branch)
        self.assertIn("wait_imu_to_init:=true", rgbd_branch)
        rgbd = texts["gz_start_rtabmap_rgbd.sh"]
        self.assertIn("approx_sync:=true", rgbd)
        self.assertIn("--Vis/DepthAsMask true", rgbd)
        self.assertIn("wait_imu_to_init:=true", rgbd)
        self.assertNotIn("--Grid/3D", rgbd)


if __name__ == "__main__":
    unittest.main()
