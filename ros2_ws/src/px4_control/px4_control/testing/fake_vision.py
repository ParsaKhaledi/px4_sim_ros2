"""TEST SUBSTITUTE: republish a pose topic as ``/rtabmap/odom``.

This is not RTAB-Map and it does not estimate anything. It exists so a SITL
run can exercise ``ESTIMATION_MODE=vision`` when the SLAM node is not up.
The default input is ``/ground_truth/odom`` (ENU), which the flight harness
fills from Gazebo's true pose.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


class FakeVisionSource(Node):
    """Copy ground-truth odometry onto the vision topic, unchanged."""

    def __init__(self) -> None:
        super().__init__('fake_vision_source')
        self.declare_parameter('source_topic', '/ground_truth/odom')
        self.declare_parameter('target_topic', '/rtabmap/odom')
        source = str(self.get_parameter('source_topic').value)
        target = str(self.get_parameter('target_topic').value)
        self.get_logger().warning(
            'TEST SUBSTITUTE: republishing %s as %s. This is Gazebo ground truth, not RTAB-Map.'
            % (source, target)
        )
        self._pub = self.create_publisher(Odometry, target, 10)
        self.create_subscription(Odometry, source, self._forward, QoSProfile(depth=10))

    def _forward(self, msg: Odometry) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FakeVisionSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
