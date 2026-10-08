"""Publish the spawn frame and a ground-truth TF.

``PX4_GZ_MODEL_POSE`` (default ``-3,-1.6,0.15,0,0,3.14``) is where the model
spawns. Pose z is drop clearance above the floor. The static ``world`` ->
``spawn`` frame uses that x, y, and yaw with z = 0, so it sits on the ground
under the drone. Ground truth is broadcast as ``world`` -> ``base_link_gt``
(``GT_CHILD_FRAME``). It is not ``base_link``: RTAB-Map publishes ``odom`` ->
``base_link``, and a second parent breaks the tree.

Both are in the Gazebo ENU world (x east, y north, z up).
"""

from __future__ import annotations

import os

from sim_monitor.checks import parse_spawn_pose, quat_from_rpy

DEFAULT_POSE = "-3,-1.6,0.15,0,0,3.14"


def spawn_ground_pose(pose: tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float, float, float]:
    """``world`` -> ``spawn``: x, y, and yaw from the model pose, z = 0.

    Pose z is drop clearance, so the frame stays on the floor. Roll and pitch
    are dropped so takeoff height relative to ``spawn`` is height above the floor.
    """
    x, y, _z, _roll, _pitch, yaw = pose
    return (float(x), float(y), 0.0, 0.0, 0.0, float(yaw))


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class SpawnFrame(Node):
        def __init__(self) -> None:
            use_sim = os.environ.get("USE_SIM_TIME", "true").lower() in {"1", "true", "yes"}
            super().__init__(
                "spawn_frame",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim)],
            )
            pose_text = os.environ.get("PX4_GZ_MODEL_POSE", "") or DEFAULT_POSE
            x, y, z, roll, pitch, yaw = spawn_ground_pose(parse_spawn_pose(pose_text))
            qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
            self.static = StaticTransformBroadcaster(self)
            message = TransformStamped()
            message.header.frame_id = "world"
            message.child_frame_id = "spawn"
            message.transform.translation.x = x
            message.transform.translation.y = y
            message.transform.translation.z = z
            message.transform.rotation.x = qx
            message.transform.rotation.y = qy
            message.transform.rotation.z = qz
            message.transform.rotation.w = qw
            self.spawn = message
            self.static.sendTransform(message)
            self.broadcaster = TransformBroadcaster(self)
            self.gt_child = os.environ.get("GT_CHILD_FRAME", "") or "base_link_gt"
            topic = os.environ.get("PREFLIGHT_GT_TOPIC", "/ground_truth/odom")
            self.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)
            self.create_timer(1.0, self._republish_spawn)
            self.get_logger().info(
                f"spawn frame world -> spawn at {x:.3f} {y:.3f} {z:.3f} "
                f"rpy {roll:.3f} {pitch:.3f} {yaw:.3f}"
            )

        def _republish_spawn(self) -> None:
            self.spawn.header.stamp = self.get_clock().now().to_msg()
            self.static.sendTransform(self.spawn)

        def _on_odom(self, msg: Odometry) -> None:
            transform = TransformStamped()
            transform.header = msg.header
            if not transform.header.frame_id:
                transform.header.frame_id = "world"
            transform.child_frame_id = self.gt_child
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self.broadcaster.sendTransform(transform)

    rclpy.init()
    node = SpawnFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
