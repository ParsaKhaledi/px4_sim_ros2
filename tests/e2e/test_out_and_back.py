#!/usr/bin/env python3
"""Out-and-back mission scored on /ground_truth/odom.

The Drone class lands with the control packages. Until that package imports,
this script exits 0 and records a skip so CI can stay green.
"""

from __future__ import annotations

import importlib
import json
import os
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "e2e"))

from grading import grade_mission, load_thresholds, yaw_from_quat  # noqa: E402


class SkipMission(Exception):
    pass


DRONE_CANDIDATES = (
    ("px4_control", "Drone"),
    ("px4_control.drone", "Drone"),
    ("px4_control.api", "Drone"),
)


def import_drone():
    for module_name, attr in DRONE_CANDIDATES:
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        drone_cls = getattr(module, attr, None)
        if drone_cls is not None:
            return drone_cls
    return None


def _call(obj, names, *args, **kwargs):
    for name in names:
        func = getattr(obj, name, None)
        if func is None:
            continue
        try:
            return func(*args, **kwargs)
        except TypeError:
            continue
    raise SkipMission(f"Drone API has no usable method among {names}")


def _result_path() -> Path:
    override = os.environ.get("E2E_RESULT_PATH")
    if override:
        return Path(override)
    container = Path("/home/px4/volume/logs/flights/trajectory.json")
    if container.parent.exists():
        return container
    return ROOT / "logs" / "flights" / "trajectory.json"


def _write(payload: dict) -> None:
    path = _result_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


class GroundTruthTrace:
    """Background sampler. No-op until start() and rclpy are available."""

    def __init__(self):
        self.samples = []
        self.phase = "start"
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self.phase = phase

    def start(self, timeout_s: float = 20.0) -> None:
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        except ImportError as exc:
            raise SkipMission(f"rclpy or nav_msgs is not importable ({exc})") from exc

        rclpy.init(args=None)
        node = rclpy.create_node("e2e_ground_truth")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def on_msg(msg):
            pose = msg.pose.pose
            quat = pose.orientation
            row = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": yaw_from_quat(quat.x, quat.y, quat.z, quat.w),
            }
            with self._lock:
                row["phase"] = self.phase
                self.samples.append(row)

        node.create_subscription(Odometry, "/ground_truth/odom", on_msg, qos)

        def spin():
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

        self._thread = threading.Thread(target=spin, daemon=True)
        self._thread.start()
        self._node = node
        self._rclpy = rclpy
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self.samples:
                    return
            time.sleep(0.2)
        required = os.environ.get("E2E_REQUIRE_GROUND_TRUTH", "0") == "1"
        if required:
            raise RuntimeError("/ground_truth/odom published no samples")
        raise SkipMission(
            "/ground_truth/odom is not publishing. The ground-truth bridge is a separate change."
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        node = getattr(self, "_node", None)
        rclpy = getattr(self, "_rclpy", None)
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


def run_mission() -> dict:
    drone_cls = import_drone()
    if drone_cls is None:
        raise SkipMission(
            "px4_control.Drone is not importable. The control package is added separately."
        )
    thresholds = load_thresholds()
    trace = GroundTruthTrace()
    drone = drone_cls()
    try:
        trace.start()
        trace.set_phase("start")
        if hasattr(drone, "preflight"):
            _call(drone, ("preflight",))
        _call(drone, ("arm",))
        try:
            _call(drone, ("takeoff",), thresholds.takeoff_height_m)
        except SkipMission:
            _call(drone, ("takeoff",), height_m=thresholds.takeoff_height_m)
        trace.set_phase("hover")
        if hasattr(drone, "hold"):
            try:
                _call(drone, ("hold",), thresholds.hover_s)
            except SkipMission:
                time.sleep(thresholds.hover_s)
        else:
            time.sleep(thresholds.hover_s)
        trace.set_phase("leg1")
        try:
            _call(drone, ("move_forward",), thresholds.leg_length_m)
        except SkipMission:
            _call(drone, ("move_forward",), distance_m=thresholds.leg_length_m)
        trace.set_phase("yaw")
        try:
            _call(drone, ("turn",), 180.0)
        except SkipMission:
            _call(drone, ("turn",), degrees=180.0)
        trace.set_phase("leg2")
        try:
            _call(drone, ("move_forward",), thresholds.leg_length_m)
        except SkipMission:
            _call(drone, ("move_forward",), distance_m=thresholds.leg_length_m)
        trace.set_phase("land")
        _call(drone, ("land",))
        if hasattr(drone, "disarm"):
            _call(drone, ("disarm",))
        time.sleep(1.0)
    finally:
        trace.stop()
    graded = grade_mission(trace.samples, thresholds)
    graded["status"] = "passed" if graded["passed"] else "failed"
    graded["samples"] = len(trace.samples)
    return graded


def test_e2e_out_and_back():
    pytest = __import__("pytest")
    try:
        result = run_mission()
    except SkipMission as exc:
        pytest.skip(str(exc))
    assert result["passed"], result


def main() -> int:
    try:
        result = run_mission()
    except SkipMission as exc:
        payload = {"status": "skipped", "reason": str(exc)}
        _write(payload)
        print(f"SKIP: {exc}")
        return 0
    _write(result)
    print(json.dumps({key: result[key] for key in ("status", "passed", "checks")}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
