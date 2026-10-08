#!/usr/bin/env python3
"""Republish the right stereo camera_info as a rectified partner of the left.

Both renders use the left calibration. This node writes that same K and P
onto the right message, with P[3] = -fx * BASELINE_M (7.5 cm), and leaves header.stamp
alone so exact sync still matches the image. The left camera_info stays the
Gazebo topic; it is already that K with Tx = 0.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import geometry as geo  # noqa: E402


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = Node(
        "stereo_baseline_relay",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    profile = geo.profile_from_env()
    # This node runs in the Rtabmap container. That container does not render
    # and cannot see the host GPU, so the missing-GPU warning is left to the
    # PX4 container's SDF render.
    warning = geo.sub_hd_warning(profile)
    if warning:
        node.get_logger().warning(warning)
    publisher = node.create_publisher(CameraInfo, geo.RIGHT_INFO_OUT, qos_profile_sensor_data)

    def on_info(msg: CameraInfo) -> None:
        # Incoming K is ignored. The pair is rectified to the left calibration
        # at the active profile's resolution, so Tx uses the scaled fx.
        msg.k = geo.rectified_k(profile)
        msg.p = geo.rectified_p(right=True, profile=profile)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        publisher.publish(msg)

    node.create_subscription(CameraInfo, geo.RIGHT_INFO_IN, on_info, qos_profile_sensor_data)
    fx = geo.stereo_intrinsics(profile)["fx"]
    node.get_logger().info(
        f"profile {profile.name} stereo {profile.stereo_width}x{profile.stereo_height} "
        f"at {profile.camera_hz:g} Hz baseline {geo.BASELINE_M:.3f} m fx {fx:.4f}: "
        f"{geo.RIGHT_INFO_IN} -> {geo.RIGHT_INFO_OUT}"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
