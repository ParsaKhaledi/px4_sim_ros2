#!/usr/bin/env python3
"""Measure vision topic rates and the Gazebo real-time factor.

Runs for a wall-clock window (``use_sim_time`` stays off, so a slow sim does
not stretch the wait). Counts image topics, ``/imu``, and ``/rtabmap/odom``.
The real-time factor is how far ``/clock`` advanced over that same wall
window. Writes JSONL and one human summary line.

This does not start the simulator and does not fail a threshold. The odometry
gate lives in ``rtabmap_health_log.py``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time


IMAGE_TOPICS = (
    "/camera/stereo/left/image_raw",
    "/camera/stereo/right/image_raw",
    "/camera/rgb/image_raw",
    "/camera/depth/image_raw",
)
IMU_TOPIC = "/imu"
ODOM_TOPIC = "/rtabmap/odom"
CLOCK_TOPIC = "/clock"
TOPICS = IMAGE_TOPICS + (IMU_TOPIC, ODOM_TOPIC)


def summarize(counts: dict, window_s: float, clock_span: float | None) -> dict:
    """Rates in Hz, and real-time factor = sim seconds advanced / wall seconds."""
    window = float(window_s)
    rates = {}
    for topic in TOPICS:
        count = int(counts.get(topic, 0))
        rates[topic] = None if window <= 0.0 else count / window
    rtf = None
    if clock_span is not None and window > 0.0:
        rtf = float(clock_span) / window
    return {"window_s": window, "rates_hz": rates, "rtf": rtf, "counts": {topic: int(counts.get(topic, 0)) for topic in TOPICS}}


def _hz(rates: dict, topic: str) -> str:
    value = rates.get(topic)
    if value is None:
        return "n/a"
    return f"{value:.2f} Hz"


def format_summary(result: dict) -> str:
    rates = result["rates_hz"]
    rtf = result["rtf"]
    rtf_text = "n/a" if rtf is None else f"{rtf:.3f}"
    return (
        f"left {_hz(rates, IMAGE_TOPICS[0])}, "
        f"right {_hz(rates, IMAGE_TOPICS[1])}, "
        f"rgb {_hz(rates, IMAGE_TOPICS[2])}, "
        f"depth {_hz(rates, IMAGE_TOPICS[3])}, "
        f"imu {_hz(rates, IMU_TOPIC)}, "
        f"odom {_hz(rates, ODOM_TOPIC)}, "
        f"rtf {rtf_text} over {result['window_s']:.1f}s"
    )


def jsonl_events(result: dict) -> list[dict]:
    events = []
    for topic, rate in result["rates_hz"].items():
        events.append(
            {
                "event": "rate",
                "topic": topic,
                "hz": rate,
                "count": result["counts"][topic],
                "window_s": result["window_s"],
            }
        )
    events.append(
        {
            "event": "summary",
            "window_s": result["window_s"],
            "rtf": result["rtf"],
            "rates_hz": result["rates_hz"],
        }
    )
    return events


def write_jsonl(handle, event: dict) -> None:
    handle.write(json.dumps(event, sort_keys=True) + "\n")
    handle.flush()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Report camera, IMU, odom, and clock rates.")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--output", default="vision_rate.jsonl")
    args = parser.parse_args(argv)
    if args.seconds <= 0.0:
        print("seconds must be positive", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu

    rclpy.init()
    # Wall clock on purpose. use_sim_time would wait N sim seconds.
    node = Node("vision_rate_probe")
    counts = {topic: 0 for topic in TOPICS}
    clock_span = {"first": None, "last": None}

    def count(topic):
        def _on_msg(_msg) -> None:
            counts[topic] += 1

        return _on_msg

    for topic in IMAGE_TOPICS:
        node.create_subscription(Image, topic, count(topic), qos_profile_sensor_data)
    node.create_subscription(Imu, IMU_TOPIC, count(IMU_TOPIC), qos_profile_sensor_data)
    node.create_subscription(Odometry, ODOM_TOPIC, count(ODOM_TOPIC), qos_profile_sensor_data)

    def on_clock(msg: Clock) -> None:
        stamp = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if clock_span["first"] is None:
            clock_span["first"] = stamp
        clock_span["last"] = stamp

    node.create_subscription(Clock, CLOCK_TOPIC, on_clock, qos_profile_sensor_data)
    start = time.monotonic()
    end = start + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        window = time.monotonic() - start
        span = None
        if clock_span["first"] is not None and clock_span["last"] is not None:
            span = clock_span["last"] - clock_span["first"]
        result = summarize(counts, window, span)
        output = open(args.output, "a", encoding="utf-8")
        try:
            for event in jsonl_events(result):
                write_jsonl(output, event)
        finally:
            output.close()
        print(format_summary(result))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
