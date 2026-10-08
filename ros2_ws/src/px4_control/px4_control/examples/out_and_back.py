"""Out-and-back mission used by the end-to-end check.

    takeoff(2), hold(10), move_forward(0.3), turn(180), move_forward(0.3), land()
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from px4_control.drone import Drone


def run(drone: Drone) -> None:
    drone.takeoff(2.0)
    drone.hold(10.0)
    drone.move_forward(0.3)
    drone.turn(180.0)
    drone.move_forward(0.3)
    drone.land()


def main() -> None:
    rclpy.init()
    node = Node('out_and_back')
    try:
        run(Drone(node))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
