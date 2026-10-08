#!/usr/bin/env python3
"""Log RTAB-Map tracking health as JSONL.

Subscribes to ``/rtabmap/info`` (rtabmap_msgs/Info). Each map update is one
line. Tracking loss is counted from the ``Odometry/Inliers/`` statistic that
RTAB-Map copies into that message: a run of updates below Vis/MinInliers
(default 20) is one loss, and the next good update closes it with a duration.
Loop closures are the message's ``loop_closure_id``.

``--fail-on-loss`` (or VISION_FAIL_ON_LOSS=true) exits 1 if any loss was seen,
which is what a headless flight check wants. Without the flag the process
exits 0 after Ctrl-C so it can sit beside a manual run.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone


# RTAB-Map copies odometry registration into /rtabmap/info under this key
# (the trailing slash is how an empty unit is formatted in Statistics.h).
INLIER_KEYS = (
    "Odometry/Inliers/",
    "Odometry/Inliers",
    "Vis/Inliers",
    "Kp/CurrentFrame/inliers",
)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def wall_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def stats_dict(keys, values) -> dict:
    return {str(key): float(value) for key, value in zip(keys, values)}


def pick_inliers(stats: dict):
    for key in INLIER_KEYS:
        if key in stats:
            return key, float(stats[key])
    for key, value in stats.items():
        lowered = key.lower().rstrip("/")
        if lowered.endswith("inliers"):
            return key, float(value)
    return None, None


def lost_from_stats(stats: dict, inliers, min_inliers: float):
    """True when this update is a tracking loss.

    An explicit ``*Lost`` statistic wins. Otherwise a published inlier count
    below the RTAB-Map minimum is a loss. Missing inliers are not a loss:
    early messages sometimes have no odometry block yet.
    """
    for key, value in stats.items():
        if key.rstrip("/").endswith("Lost"):
            return float(value) >= 0.5
    if inliers is None:
        return False
    return inliers < min_inliers


class TrackingLog:
    def __init__(self, min_inliers: float = 20.0):
        self.min_inliers = min_inliers
        self.loss_count = 0
        self.lost = False
        self.loss_started = None
        self.seen_loops = set()
        self.loop_count = 0
        self.last_stamp = None

    def update(self, stamp, wall, ref_id, loop_closure_id, proximity_id, stats) -> list:
        key, inliers = pick_inliers(stats)
        lost_now = lost_from_stats(stats, inliers, self.min_inliers)
        self.last_stamp = stamp
        events = [
            {
                "stamp": stamp,
                "wall_time": wall,
                "event": "frame",
                "ref_id": ref_id,
                "inliers": inliers,
                "inliers_key": key,
                "loop_closure_id": int(loop_closure_id),
                "proximity_detection_id": int(proximity_id),
                "tracking": "lost" if lost_now else "ok",
                "loss_count": self.loss_count,
            }
        ]
        if lost_now and not self.lost:
            self.lost = True
            self.loss_count += 1
            self.loss_started = stamp
            events.append(
                {
                    "stamp": stamp,
                    "wall_time": wall,
                    "event": "tracking_lost_start",
                    "loss_count": self.loss_count,
                }
            )
        elif not lost_now and self.lost:
            events.append(self._loss_end(stamp, wall, open_ended=False))
        if loop_closure_id and (ref_id, int(loop_closure_id)) not in self.seen_loops:
            self.seen_loops.add((ref_id, int(loop_closure_id)))
            self.loop_count += 1
            events.append(
                {
                    "stamp": stamp,
                    "wall_time": wall,
                    "event": "loop_closure",
                    "loop_closure_id": int(loop_closure_id),
                    "ref_id": ref_id,
                    "loop_count": self.loop_count,
                }
            )
        return events

    def _loss_end(self, stamp, wall, open_ended: bool) -> dict:
        duration = None if self.loss_started is None else float(stamp) - float(self.loss_started)
        self.lost = False
        self.loss_started = None
        return {
            "stamp": stamp,
            "wall_time": wall,
            "event": "tracking_lost_end",
            "loss_count": self.loss_count,
            "duration_s": duration,
            "open": open_ended,
        }

    def finish(self, wall) -> list:
        if not self.lost:
            return []
        stamp = self.last_stamp if self.last_stamp is not None else 0.0
        return [self._loss_end(stamp, wall, open_ended=True)]

    def summary(self, wall) -> dict:
        return {
            "stamp": self.last_stamp,
            "wall_time": wall,
            "event": "summary",
            "loss_count": self.loss_count,
            "loop_count": self.loop_count,
            "tracking": "lost" if self.lost else "ok",
        }


def write_jsonl(handle, event: dict) -> None:
    handle.write(json.dumps(event, sort_keys=True) + "\n")
    handle.flush()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Log /rtabmap/info tracking health as JSONL.")
    parser.add_argument("--output", default="rtabmap_health.jsonl")
    parser.add_argument("--min-inliers", type=float, default=float(os.environ.get("VISION_MIN_INLIERS", "20")))
    parser.add_argument("--fail-on-loss", action="store_true")
    args = parser.parse_args(argv)
    fail_on_loss = args.fail_on_loss or os.environ.get("VISION_FAIL_ON_LOSS", "").lower() in ("1", "true", "yes")

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rtabmap_msgs.msg import Info

    rclpy.init()
    node = Node(
        "rtabmap_health_log",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    tracker = TrackingLog(args.min_inliers)
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    output = open(args.output, "a", encoding="utf-8")

    def on_info(msg: Info) -> None:
        stats = stats_dict(msg.stats_keys, msg.stats_values)
        events = tracker.update(
            stamp_seconds(msg.header.stamp),
            wall_now(),
            int(msg.ref_id),
            int(msg.loop_closure_id),
            int(msg.proximity_detection_id),
            stats,
        )
        for event in events:
            write_jsonl(output, event)

    node.create_subscription(Info, "/rtabmap/info", on_info, qos)
    node.get_logger().info(f"logging /rtabmap/info to {args.output}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for event in tracker.finish(wall_now()):
            write_jsonl(output, event)
        write_jsonl(output, tracker.summary(wall_now()))
        output.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fail_on_loss and tracker.loss_count:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
