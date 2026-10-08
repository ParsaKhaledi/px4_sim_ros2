"""Publish a measured real-time factor on /sim/real_time_factor.

Gazebo's own ``real_time_factor`` field under-reads on a software-rendered
CPU sim (about 0.03 while sim time advanced at about 0.21 of wall time).
This node computes Δsim_time/Δwall_time over a sliding window
(``SIM_RTF_WINDOW_S``, default 5 s) from the stats message's ``sim_time``
and a steady wall clock. The raw gz field is also published on
``/sim/real_time_factor_gz``. The node uses wall time so it can keep
publishing when /clock is stopped.
"""

from __future__ import annotations

import os
import re
import subprocess
import threading
from typing import Optional

RTF_PATTERN = re.compile(r"real_time_factor:\s*([-+0-9.eE]+)")
SIM_TIME_BLOCK = re.compile(r"sim_time\s*\{([^}]*)\}", re.DOTALL)
SIM_SEC = re.compile(r"sec:\s*(-?\d+)")
SIM_NSEC = re.compile(r"nsec:\s*(-?\d+)")
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


def parse_stats_message(text: str) -> tuple[float | None, float | None]:
    """Return ``(sim_time_s, gz_real_time_factor)`` from one stats message."""
    sim_time = None
    block = SIM_TIME_BLOCK.search(text)
    if block is not None:
        body = block.group(1)
        sec = SIM_SEC.search(body)
        if sec is not None:
            nsec = SIM_NSEC.search(body)
            sim_time = int(sec.group(1)) + (int(nsec.group(1)) if nsec else 0) * 1e-9
    return sim_time, iter_real_time_factors(text)


def windowed_rtf(samples: list[tuple[float, float]], window_s: float) -> float | None:
    """Real-time factor Δsim/Δwall over the newest ``window_s`` of wall time.

    ``samples`` are ``(wall_s, sim_s)`` in time order. The span may be shorter
    than the window while the buffer is still filling.
    """
    if len(samples) < 2 or window_s <= 0.0:
        return None
    end_wall, end_sim = samples[-1]
    start_wall, start_sim = samples[0]
    for wall, sim in samples:
        if end_wall - wall <= window_s:
            start_wall, start_sim = wall, sim
            break
    delta_wall = end_wall - start_wall
    if delta_wall <= 1e-6:
        return None
    return (end_sim - start_sim) / delta_wall


def rtf_window_s(environ: dict[str, str] | None = None) -> float:
    """Sliding window from ``SIM_RTF_WINDOW_S`` (default 5 s)."""
    env = os.environ if environ is None else environ
    raw = env.get("SIM_RTF_WINDOW_S", "")
    if raw == "":
        return 5.0
    return float(raw)


def consume_stats_line(buffer: str, line: str) -> tuple[str, tuple[float | None, float | None] | None]:
    """Fold one echo line into ``buffer``. A blank line ends a message."""
    if line.strip() == "":
        if not buffer.strip():
            return "", None
        return "", parse_stats_message(buffer)
    updated = buffer + line
    sim_time, gz_rtf = parse_stats_message(updated)
    if sim_time is not None and gz_rtf is not None:
        return "", (sim_time, gz_rtf)
    return updated, None


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
            self.gz_publisher = self.create_publisher(Float64, "/sim/real_time_factor_gz", qos)
            self._window_s = rtf_window_s()
            self._process = None
            self._topic = None
            self._latest: Optional[float] = None
            self._gz_rtf: Optional[float] = None
            self._samples: list[tuple[float, float]] = []
            self._lock = threading.Lock()
            self._logs = 0
            self.create_timer(2.0, self._ensure_stream)
            self.create_timer(0.5, self._publish)
            self.get_logger().info(
                f"publishing /sim/real_time_factor over {self._window_s:.1f}s "
                "(raw gz field on /sim/real_time_factor_gz)"
            )

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
            import time
            buffer = ""
            for chunk in self._process.stdout:
                buffer, parsed = consume_stats_line(buffer, chunk)
                if parsed is None:
                    continue
                sim_time, gz_rtf = parsed
                wall = time.monotonic()
                with self._lock:
                    self._gz_rtf = gz_rtf
                    if sim_time is not None:
                        self._samples.append((wall, sim_time))
                        while (
                            len(self._samples) > 2
                            and wall - self._samples[0][0] > self._window_s
                        ):
                            self._samples.pop(0)
                        self._latest = windowed_rtf(self._samples, self._window_s)

        def _publish(self) -> None:
            with self._lock:
                value = self._latest
                gz_value = self._gz_rtf
            if gz_value is not None:
                raw = Float64()
                raw.data = gz_value
                self.gz_publisher.publish(raw)
            if value is None:
                return
            message = Float64()
            message.data = value
            self.publisher.publish(message)
            self._logs += 1
            if self._logs % 10 == 1:
                raw_text = f"{gz_value:.3f}" if gz_value is not None else "n/a"
                self.get_logger().info(f"real_time_factor={value:.3f} (gz field {raw_text})")

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
