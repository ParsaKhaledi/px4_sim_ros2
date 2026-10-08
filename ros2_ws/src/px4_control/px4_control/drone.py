"""Blocking mission API.

A script looks like the old MAVROS helpers::

    drone = Drone(node)
    drone.takeoff(2.0)
    drone.hold(10.0)
    drone.move_forward(0.3)
    drone.turn(180.0)
    drone.land()

``turn`` is positive counter-clockwise when viewed from above. Timeouts
use the node clock, which is sim time when ``use_sim_time`` is set.
"""

from __future__ import annotations

import math
from typing import TypeVar

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from px4_control_interfaces.action import GoTo, Hold, Land, Takeoff
from px4_control_interfaces.srv import Arm, SetMode

GoalT = TypeVar('GoalT')
ResultT = TypeVar('ResultT')


class Drone:
    """Client for the ``px4_control`` action and service servers."""

    def __init__(self, node: Node, prefix: str = '/px4_control') -> None:
        self._node = node
        self._prefix = prefix.rstrip('/')
        self._arm = node.create_client(Arm, f'{self._prefix}/arm')
        self._mode = node.create_client(SetMode, f'{self._prefix}/set_mode')
        self._takeoff = ActionClient(node, Takeoff, f'{self._prefix}/takeoff')
        self._land = ActionClient(node, Land, f'{self._prefix}/land')
        self._goto = ActionClient(node, GoTo, f'{self._prefix}/goto')
        self._hold = ActionClient(node, Hold, f'{self._prefix}/hold')

    def arm(self, timeout: float = 30.0) -> None:
        self._call_service(self._arm, Arm.Request(arm=True), timeout)

    def disarm(self, timeout: float = 30.0) -> None:
        self._call_service(self._arm, Arm.Request(arm=False), timeout)

    def set_mode(self, mode: str, timeout: float = 15.0) -> None:
        self._call_service(self._mode, SetMode.Request(mode=mode), timeout)

    def takeoff(self, height: float, timeout: float = 180.0) -> None:
        goal = Takeoff.Goal()
        goal.height = float(height)
        self._call_action(self._takeoff, goal, timeout)

    def land(self, timeout: float = 180.0) -> None:
        self._call_action(self._land, Land.Goal(), timeout)

    def hold(self, seconds: float, timeout: float | None = None) -> None:
        goal = Hold.Goal()
        goal.duration_sec = float(seconds)
        limit = timeout if timeout is not None else max(60.0, float(seconds) + 120.0)
        self._call_action(self._hold, goal, limit)

    def move_forward(self, metres: float, timeout: float = 120.0) -> None:
        """Fly ``metres`` along the current heading. Level, yaw-only."""
        goal = GoTo.Goal()
        goal.frame = GoTo.Goal.FRAME_BODY
        goal.x = float(metres)
        goal.y = 0.0
        goal.z = 0.0
        goal.yaw = 0.0
        goal.yaw_valid = False
        self._call_action(self._goto, goal, timeout)

    def turn(self, degrees: float, timeout: float = 180.0) -> None:
        """Yaw in place. Positive degrees are counter-clockwise from above."""
        goal = GoTo.Goal()
        goal.frame = GoTo.Goal.FRAME_BODY
        goal.x = 0.0
        goal.y = 0.0
        goal.z = 0.0
        goal.yaw = math.radians(float(degrees))
        goal.yaw_valid = True
        self._call_action(self._goto, goal, timeout)

    def goto(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float | None = None,
        frame: str = 'enu',
        timeout: float = 180.0,
    ) -> None:
        """Go to a pose. ``frame`` is ``enu`` or ``body``. ``yaw`` is NED radians
        in ENU, or a relative counter-clockwise angle in the body frame.
        """
        goal = GoTo.Goal()
        goal.frame = GoTo.Goal.FRAME_BODY if frame == 'body' else GoTo.Goal.FRAME_LOCAL_ENU
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.yaw_valid = yaw is not None
        goal.yaw = 0.0 if yaw is None else float(yaw)
        self._call_action(self._goto, goal, timeout)

    def _call_service(self, client, request, timeout: float) -> None:
        if not client.wait_for_service(timeout_sec=min(timeout, 15.0)):
            raise TimeoutError(f'{client.srv_name} is not available')
        future = client.call_async(request)
        self._wait_future(future, timeout, client.srv_name)
        response = future.result()
        if response is None or not response.success:
            message = '' if response is None else response.message
            raise RuntimeError(message or f'{client.srv_name} failed')

    def _call_action(self, client: ActionClient, goal, timeout: float) -> None:
        if not client.wait_for_server(timeout_sec=min(timeout, 15.0)):
            raise TimeoutError(f'{client._action_name} is not available')
        send = client.send_goal_async(goal)
        self._wait_future(send, timeout, 'goal')
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        result_future = handle.get_result_async()
        self._wait_future(result_future, timeout, 'result')
        wrapped = result_future.result()
        result = None if wrapped is None else wrapped.result
        if result is None or not result.success:
            message = '' if result is None else result.message
            raise RuntimeError(message or 'action failed')

    def _now_s(self) -> float:
        """Node clock. Sim time when the node was started with use_sim_time."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _wait_future(self, future, timeout: float, label: str) -> None:
        """Wait on the node clock.

        The first ``/clock`` sample can jump from 0 to the current sim time.
        The budget starts at that first valid sample, so a clock that is
        already past ``timeout`` does not expire the call immediately.
        """
        start: float | None = None
        use_sim = bool(self._node.get_parameter('use_sim_time').value)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            now = self._now_s()
            if start is None:
                if use_sim and now <= 0.0:
                    continue
                start = now
            elif now - start >= timeout:
                raise TimeoutError(f'timed out waiting for {label}')
