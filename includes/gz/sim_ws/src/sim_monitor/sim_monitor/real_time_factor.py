"""Publish Gazebo's real-time factor on /sim/real_time_factor.

The value is read from the gz world statistics topic
``/world/<name>/stats`` (field ``real_time_factor``). The node uses wall time
so it can keep publishing when /clock is stopped.
"""

from __future__ import annotations

import re
import subprocess
import threading
from typing import Optional

RTF_PATTERN = re.compile(r"real_time_factor:\s*([-+0-9.eE]+)")
STATS_PATTERN = re.compile(r"^/world/[^/]+/stats$")


def find_stats_topic() -> str | None:
    """Return the first gz world stats topic, or None if Gazebo is not up."""
    try:
        completed = subprocess.run(
            ["gz", "topic", "-l"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in completed.stdout.splitlines():
        topic = line.strip()
        if STATS_PATTERN.match(topic):
            return topic
    return None


def iter_real_time_factors(stream) -> float | None:
    """Parse one real_time_factor value from a gz topic text stream chunk."""
    match = RTF_PATTERN.search(stream)
    if match is None:
        return None
    return float(match.group(1))


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64

    class RealTimeFactor(Node):
        def __init__(self) -> None:
            super().__init__("sim_real_time_factor")
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.publisher = self.create_publisher(Float64, "/sim/real_time_factor", qos)
            self._process = None
            self._topic = None
            self._latest: Optional[float] = None
            self._lock = threading.Lock()
            self._logs = 0
            self.create_timer(2.0, self._ensure_stream)
            self.create_timer(0.5, self._publish)
            self.get_logger().info("publishing /sim/real_time_factor")

        def _ensure_stream(self) -> None:
            if self._process is not None and self._process.poll() is None:
                return
            topic = find_stats_topic()
            if topic is None:
                self.get_logger().warning("waiting for a Gazebo /world/*/stats topic")
                return
            self._topic = topic
            self._process = subprocess.Popen(
                ["gz", "topic", "-e", "-t", topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            thread = threading.Thread(target=self._read, daemon=True)
            thread.start()
            self.get_logger().info(f"reading real_time_factor from {topic}")

        def _read(self) -> None:
            assert self._process is not None and self._process.stdout is not None
            buffer = ""
            for chunk in self._process.stdout:
                buffer += chunk
                value = iter_real_time_factors(buffer)
                if value is None:
                    continue
                buffer = ""
                with self._lock:
                    self._latest = value

        def _publish(self) -> None:
            with self._lock:
                value = self._latest
            if value is None:
                return
            message = Float64()
            message.data = value
            self.publisher.publish(message)
            self._logs += 1
            if self._logs % 10 == 1:
                self.get_logger().info(f"real_time_factor={value:.3f}")

    rclpy.init()
    node = RealTimeFactor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
