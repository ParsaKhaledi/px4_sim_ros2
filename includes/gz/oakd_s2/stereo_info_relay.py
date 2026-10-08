#!/usr/bin/env python3
"""Republish the right stereo camera_info with the OAK-D S2 baseline.

Gazebo Harmonic writes ``<projection><tx>`` into CameraInfo P[3], and the SDF
sets that to -fx * 0.075. This node publishes the same correction on a second
topic so RTAB-Map still sees the baseline if a Gazebo build leaves Tx at 0.
If P[3] is already right, the value is written again and does not change.
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
    publisher = node.create_publisher(CameraInfo, geo.RIGHT_INFO_OUT, qos_profile_sensor_data)
    warned = {"empty": False}

    def on_info(msg: CameraInfo) -> None:
        corrected = geo.corrected_projection(msg.p, msg.k)
        if corrected is None:
            if not warned["empty"]:
                node.get_logger().warning("right camera_info has no fx yet; waiting")
                warned["empty"] = True
            return
        msg.p = corrected
        publisher.publish(msg)

    node.create_subscription(CameraInfo, geo.RIGHT_INFO_IN, on_info, qos_profile_sensor_data)
    node.get_logger().info(
        f"baseline {geo.BASELINE_M:.3f} m: {geo.RIGHT_INFO_IN} -> {geo.RIGHT_INFO_OUT}"
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
