"""Publish the camera static TF tree parsed from the model SDFs.

robot_state_publisher used to publish the same children from a hand-written
URDF. That URDF no longer carries those joints, so each frame has one parent.
"""

from __future__ import annotations

import os

from sim_monitor.camera_extrinsics import StaticTF, load_camera_transforms
from sim_monitor.imu_frames import quat_from_rpy


def quaternion_xyzw(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = quat_from_rpy(rpy[0], rpy[1], rpy[2])
    return (float(x), float(y), float(z), float(w))


def describe(transforms: list[StaticTF]) -> str:
    lines = []
    for transform in transforms:
        xyz = " ".join(f"{value:.4f}" for value in transform.xyz)
        rpy = " ".join(f"{value:.4f}" for value in transform.rpy)
        lines.append(f"{transform.parent} -> {transform.child} xyz=({xyz}) rpy=({rpy})")
    return "\n".join(lines)


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from tf2_ros import StaticTransformBroadcaster

    transforms = load_camera_transforms()

    class CameraFrames(Node):
        def __init__(self) -> None:
            use_sim = os.environ.get("USE_SIM_TIME", "true").lower() in {"1", "true", "yes"}
            super().__init__(
                "camera_tf",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim)],
            )
            broadcaster = StaticTransformBroadcaster(self)
            stamp = self.get_clock().now().to_msg()
            messages = []
            for transform in transforms:
                message = TransformStamped()
                message.header.stamp = stamp
                message.header.frame_id = transform.parent
                message.child_frame_id = transform.child
                message.transform.translation.x = transform.xyz[0]
                message.transform.translation.y = transform.xyz[1]
                message.transform.translation.z = transform.xyz[2]
                qx, qy, qz, qw = quaternion_xyzw(transform.rpy)
                message.transform.rotation.x = qx
                message.transform.rotation.y = qy
                message.transform.rotation.z = qz
                message.transform.rotation.w = qw
                messages.append(message)
            broadcaster.sendTransform(messages)
            self.get_logger().info("camera static TF\n" + describe(transforms))

    rclpy.init()
    node = CameraFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
