"""Out-and-back mission used by the end-to-end check.

    takeoff(2), hold(10), move_forward(0.3), turn(180), move_forward(0.3), land()
"""

from __future__ import annotations

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from px4_control.camera_type import parse_camera_type
from px4_control.drone import Drone


def run(drone: Drone) -> None:
    drone.takeoff(2.0)
    drone.hold(10.0)
    drone.move_forward(0.3)
    drone.turn(180.0)
    drone.move_forward(0.3)
    drone.land()


def main() -> None:
    try:
        camera = parse_camera_type(sys.argv[1:], os.environ)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(2) from exc
    rclpy.init()
    use_sim = os.environ.get('USE_SIM_TIME', 'true').lower() not in ('0', 'false', 'no')
    node = Node(
        'out_and_back',
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim)],
    )
    node.get_logger().info(f'out_and_back CameraType={camera}')
    try:
        run(Drone(node))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
