#!/usr/bin/env python3
"""Runtime PASS/FAIL check for the OAK-D S2 vision pipeline.

Writes one JSONL record per check. This does not start the simulator. Run it
in a container that already has the bridge, robot_state_publisher, and
RTAB-Map up. Exit 0 only when every selected check passes.

``--mode`` follows CameraType: stereo, rgbd, or all.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "includes" / "gz" / "oakd_s2"))

import geometry as geo  # noqa: E402


TRANSLATION_TOL_M = 0.01
ROTATION_TOL_RAD = math.radians(5.0)
TX_TOL_PX = 0.5
FX_TOL_PX = 1.0


def record(handle, name: str, ok: bool, **detail) -> dict:
    event = {"check": name, "pass": bool(ok), **detail}
    handle.write(json.dumps(event, sort_keys=True) + "\n")
    handle.flush()
    return event


def quat_angle(a, b) -> float:
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def close_vec(got, expected, tol) -> bool:
    return all(abs(got[i] - expected[i]) <= tol for i in range(3))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check stereo, IMU, TF, and RTAB-Map topics.")
    parser.add_argument("--output", default="vision_check.jsonl")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--mode", default=os.environ.get("CameraType", "all"))
    args = parser.parse_args(argv)
    mode = args.mode.lower()
    if mode in ("rgbd", "rgb-d"):
        mode = "rgbd"

    output = open(args.output, "w", encoding="utf-8")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, Imu
        from nav_msgs.msg import Odometry
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        record(output, "ros_import", False, error=str(exc))
        output.close()
        return 2

    rclpy.init()
    node = Node(
        "vision_pipeline_check",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    results = []

    def wait_for(msg_type, topic):
        box = {}
        sub = node.create_subscription(
            msg_type, topic, lambda msg: box.setdefault("msg", msg), qos_profile_sensor_data
        )
        deadline = time.monotonic() + args.timeout
        while "msg" not in box and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(sub)
        return box.get("msg")

    def check_image(name, topic, frame):
        msg = wait_for(Image, topic)
        ok = msg is not None and msg.header.frame_id == frame
        results.append(
            record(
                output,
                name,
                ok,
                topic=topic,
                frame_id=None if msg is None else msg.header.frame_id,
                expected_frame=frame,
            )
        )

    def check_fx(name, topic, frame, expected_fx):
        msg = wait_for(CameraInfo, topic)
        if msg is None:
            results.append(record(output, name, False, topic=topic, error="no message"))
            return None
        fx = float(msg.k[0])
        ok = msg.header.frame_id == frame and abs(fx - expected_fx) <= FX_TOL_PX
        results.append(
            record(
                output,
                name,
                ok,
                topic=topic,
                frame_id=msg.header.frame_id,
                expected_frame=frame,
                fx=fx,
                expected_fx=expected_fx,
            )
        )
        return msg

    mount = geo.mount_from_env()
    if mode in ("stereo", "all"):
        check_image("stereo_left_image", "/camera/stereo/left/image_raw", geo.STEREO_LEFT_OPTICAL)
        check_image("stereo_right_image", "/camera/stereo/right/image_raw", geo.STEREO_RIGHT_OPTICAL)
        check_fx(
            "stereo_left_info",
            "/camera/stereo/left/camera_info",
            geo.STEREO_LEFT_OPTICAL,
            geo.LEFT_INTRINSICS["fx"],
        )
        raw = check_fx(
            "stereo_right_info_raw",
            geo.RIGHT_INFO_IN,
            geo.STEREO_RIGHT_OPTICAL,
            geo.RIGHT_INTRINSICS["fx"],
        )
        corrected = wait_for(CameraInfo, geo.RIGHT_INFO_OUT)
        if corrected is None or raw is None:
            results.append(
                record(output, "stereo_right_tx", False, topic=geo.RIGHT_INFO_OUT, error="no message")
            )
        else:
            fx = float(corrected.k[0]) or float(raw.k[0])
            expected = geo.stereo_tx(fx)
            got = float(corrected.p[3])
            results.append(
                record(
                    output,
                    "stereo_right_tx",
                    abs(got - expected) <= TX_TOL_PX and corrected.header.frame_id == geo.STEREO_RIGHT_OPTICAL,
                    topic=geo.RIGHT_INFO_OUT,
                    frame_id=corrected.header.frame_id,
                    p3=got,
                    expected_p3=expected,
                    raw_p3=float(raw.p[3]),
                    fx=fx,
                )
            )
    if mode in ("rgbd", "all"):
        check_image("rgb_image", "/camera/rgb/image_raw", geo.RGB_OPTICAL)
        check_image("depth_image", "/camera/depth/image_raw", geo.RGB_OPTICAL)
        color = geo.color_intrinsics()
        rgb_info = check_fx("rgb_info", "/camera/rgb/camera_info", geo.RGB_OPTICAL, color["fx"])
        depth_info = check_fx("depth_info", "/camera/depth/camera_info", geo.RGB_OPTICAL, color["fx"])
        if rgb_info is None or depth_info is None:
            results.append(record(output, "depth_aligned_to_rgb", False, error="missing camera_info"))
        else:
            same_k = all(abs(float(rgb_info.k[i]) - float(depth_info.k[i])) <= FX_TOL_PX for i in range(9))
            same_frame = rgb_info.header.frame_id == depth_info.header.frame_id == geo.RGB_OPTICAL
            results.append(
                record(
                    output,
                    "depth_aligned_to_rgb",
                    same_k and same_frame,
                    rgb_frame=rgb_info.header.frame_id,
                    depth_frame=depth_info.header.frame_id,
                )
            )

    imu = wait_for(Imu, geo.IMU_TOPIC)
    results.append(
        record(
            output,
            "imu",
            imu is not None and imu.header.frame_id == geo.IMU_FRAME,
            topic=geo.IMU_TOPIC,
            frame_id=None if imu is None else imu.header.frame_id,
            expected_frame=geo.IMU_FRAME,
        )
    )

    buffer = Buffer()
    TransformListener(buffer, node)
    tf_pairs = [("imu_link", geo.IMU_FRAME, "imu")]
    if mode in ("stereo", "all"):
        tf_pairs.append(("stereo_left_optical", geo.STEREO_LEFT_OPTICAL, "stereo_left"))
        tf_pairs.append(("stereo_right_optical", geo.STEREO_RIGHT_OPTICAL, "stereo_right"))
    if mode in ("rgbd", "all"):
        tf_pairs.append(("rgb_optical", geo.RGB_OPTICAL, "rgb"))

    def lookup(target, source):
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if buffer.can_transform(target, source, rclpy.time.Time()):
                return buffer.lookup_transform(target, source, rclpy.time.Time())
        return None

    for name, frame, sensor in tf_pairs:
        transform = lookup(geo.BASE_FRAME, frame)
        expected = geo.optical_origin_in_base(sensor, mount) if sensor != "imu" else geo.optical_origin_in_base("imu", mount)
        # IMU origin uses the same helper; its frame is not optical, but the
        # joint translation is the sensor origin. Orientation is checked only
        # for optical frames.
        if sensor == "imu":
            expected = geo.optical_origin_in_base("imu", mount)
        if transform is None:
            results.append(record(output, f"tf_{name}", False, frame=frame, error="no transform"))
            continue
        got = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        ok = close_vec(got, expected, TRANSLATION_TOL_M)
        detail = {"frame": frame, "translation": list(got), "expected_translation": list(expected)}
        if sensor != "imu":
            quat = transform.transform.rotation
            got_q = (quat.x, quat.y, quat.z, quat.w)
            expected_q = geo.optical_quaternion_in_base(mount)
            angle = quat_angle(got_q, expected_q)
            ok = ok and angle <= ROTATION_TOL_RAD
            detail["rotation_error_rad"] = angle
        results.append(record(output, f"tf_{name}", ok, **detail))

    if mode in ("stereo", "all"):
        baseline = lookup(geo.STEREO_LEFT_OPTICAL, geo.STEREO_RIGHT_OPTICAL)
        if baseline is None:
            results.append(record(output, "tf_stereo_baseline", False, error="no transform"))
        else:
            got_x = baseline.transform.translation.x
            results.append(
                record(
                    output,
                    "tf_stereo_baseline",
                    abs(got_x - geo.BASELINE_M) <= TRANSLATION_TOL_M,
                    x=got_x,
                    expected_x=geo.BASELINE_M,
                )
            )

    odom = wait_for(Odometry, "/rtabmap/odom")
    results.append(
        record(
            output,
            "rtabmap_odom",
            odom is not None,
            topic="/rtabmap/odom",
            frame_id=None if odom is None else odom.header.frame_id,
        )
    )

    failed = [item["check"] for item in results if not item["pass"]]
    record(output, "summary", not failed, failed=failed, mode=mode)
    output.close()
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
