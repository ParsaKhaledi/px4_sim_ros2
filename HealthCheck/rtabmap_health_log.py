#!/usr/bin/env python3
"""Log RTAB-Map tracking health as JSONL.

Prefers ``/rtabmap/odom_info`` (``rtabmap_msgs/OdomInfo``). Each message is one
processed frame: ``lost``, ``features``, and ``inliers``. ``/rtabmap/info``
still supplies loop closures. It is also the tracking fallback until the first
``odom_info`` arrives, using the ``Odometry/Inliers/`` statistic. After that,
info stats are not counted again, so the two topics are not mixed into one run.

The summary line scores five gates. Thresholds come from the environment
(``.env`` fills anything unset; the process environment wins):

- ``VISION_MAX_LOST_STREAK`` (default 3): longest run of ``lost`` frames.
- ``VISION_MAX_RECOVERY_FRAMES`` (default 2): for each lost run, how many
  frames elapse from that loss until the first frame that is not lost. The
  metric is the worst run. A run that never reaches a non-lost frame fails
  even when it is shorter than the cap.
- ``VISION_MIN_MEDIAN_FEATURES``: median of ``features``. Default 120.
  ``VISION_PROFILE=cpu`` uses 40 unless the variable is set.
- ``VISION_MIN_INLIERS``: floor on frames that are not lost. Default 20.
  ``VISION_PROFILE=cpu`` uses 15 unless the variable is set. Lost frames stay
  in the distribution, and the streak and recovery gates already cover them.
  The first frame after an automatic odometry reset is not lost. It starts a
  new local map and reports 0 inliers, so it stays in the distribution and is
  left out of this gate.
- ``VISION_MIN_ODOM_HZ`` (default 7): odometry rate from ``odom_info`` header
  stamps, which are simulation time (what PX4's EKF sees). The wall-clock
  rate and ``wall_hz / sim_hz`` are reported beside it and are not gated.

``--fail-on-loss`` or ``VISION_FAIL_ON_LOSS`` exits 1 when any gate fails.
Without that, the process exits 0 after Ctrl-C so it can sit beside a manual
run. The summary is still written either way.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path


# RTAB-Map copies odometry registration into /rtabmap/info under this key
# (the trailing slash is how an empty unit is formatted in Statistics.h).
INLIER_KEYS = (
    "Odometry/Inliers/",
    "Odometry/Inliers",
    "Vis/Inliers",
    "Kp/CurrentFrame/inliers",
)

DEFAULT_MAX_LOST_STREAK = 3
DEFAULT_MAX_RECOVERY_FRAMES = 2
DEFAULT_MIN_MEDIAN_FEATURES = 120
DEFAULT_MIN_INLIERS = 20
CPU_MIN_MEDIAN_FEATURES = 40
CPU_MIN_INLIERS = 15
DEFAULT_MIN_ODOM_HZ = 7

METRIC_NAMES = ("lost_streak", "recovery_frames", "median_features", "inliers", "odom_hz")


@dataclass(frozen=True)
class VisionThresholds:
    max_lost_streak: int = DEFAULT_MAX_LOST_STREAK
    max_recovery_frames: int = DEFAULT_MAX_RECOVERY_FRAMES
    min_median_features: float = DEFAULT_MIN_MEDIAN_FEATURES
    min_inliers: float = DEFAULT_MIN_INLIERS
    min_odom_hz: float = DEFAULT_MIN_ODOM_HZ


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def wall_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def stats_dict(keys, values) -> dict:
    return {str(key): float(value) for key, value in zip(keys, values)}


def json_number(value):
    if value is None:
        return None
    number = float(value)
    if number.is_integer():
        return int(number)
    return number


def load_env_file(path: Path) -> None:
    """Fill unset variables from a .env file. Existing environment wins."""
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if key.startswith("export "):
            key = key[len("export ") :].strip()
        value = value.strip().strip('"').strip("'")
        os.environ.setdefault(key, value)


def load_repo_env() -> None:
    candidates = [Path.cwd() / ".env", Path(__file__).resolve().parents[1] / ".env"]
    for path in candidates:
        load_env_file(path)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return float(default)
    return float(raw)


def profile_name_from_env() -> str:
    raw = os.environ.get("VISION_PROFILE", "full")
    name = "full" if raw is None or str(raw).strip() == "" else str(raw).strip().lower()
    if name in ("full", "cpu"):
        return name
    raise ValueError(f"VISION_PROFILE must be full or cpu, got {raw}")


def thresholds_from_env() -> VisionThresholds:
    """Explicit ``VISION_MIN_*`` values win over the profile defaults."""
    if profile_name_from_env() == "cpu":
        feature_default = CPU_MIN_MEDIAN_FEATURES
        inlier_default = CPU_MIN_INLIERS
    else:
        feature_default = DEFAULT_MIN_MEDIAN_FEATURES
        inlier_default = DEFAULT_MIN_INLIERS
    return VisionThresholds(
        max_lost_streak=int(_env_float("VISION_MAX_LOST_STREAK", DEFAULT_MAX_LOST_STREAK)),
        max_recovery_frames=int(_env_float("VISION_MAX_RECOVERY_FRAMES", DEFAULT_MAX_RECOVERY_FRAMES)),
        min_median_features=_env_float("VISION_MIN_MEDIAN_FEATURES", feature_default),
        min_inliers=_env_float("VISION_MIN_INLIERS", inlier_default),
        min_odom_hz=_env_float("VISION_MIN_ODOM_HZ", DEFAULT_MIN_ODOM_HZ),
    )


def fail_on_loss_enabled(flag: bool) -> bool:
    if flag:
        return True
    return os.environ.get("VISION_FAIL_ON_LOSS", "").lower() in ("1", "true", "yes")


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


def median(values: list[float]):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def interval_rate(times: list[float]):
    """Hz from the first sample to the last. None when the span is not positive."""
    if len(times) < 2:
        return None
    span = float(times[-1]) - float(times[0])
    if span <= 0.0:
        return None
    return (len(times) - 1) / span


def lost_runs(lost_flags: list[bool]) -> tuple[int, list[int], int]:
    """Longest lost streak, each closed episode length, and a still-open tail.

    An episode length is the number of consecutive lost frames from the loss
    until the next frame that is not lost. That next frame ends the episode
    and is not included in the count.
    """
    longest = 0
    current = 0
    episodes: list[int] = []
    for lost in lost_flags:
        if lost:
            current += 1
            longest = max(longest, current)
        elif current:
            episodes.append(current)
            current = 0
    return longest, episodes, current


def inlier_gate_value(samples: list[dict], index: int):
    """Inliers for one frame, or None when the floor does not apply.

    Lost frames are covered by the streak and recovery gates. The first
    frame after an automatic odometry reset is not lost. That frame starts
    a new local map and reports 0 inliers, so it is not a tracking miss.
    """
    sample = samples[index]
    inliers = sample["inliers"]
    if sample["lost"] or inliers is None:
        return None
    if float(inliers) == 0.0 and index > 0 and samples[index - 1]["lost"]:
        return None
    return float(inliers)


def score_samples(samples: list[dict], thresholds: VisionThresholds) -> dict:
    flags = [bool(sample["lost"]) for sample in samples]
    streak, episodes, open_frames = lost_runs(flags)
    recovery_value = max(episodes + ([open_frames] if open_frames else [0]))
    recovery_pass = open_frames == 0 and recovery_value <= thresholds.max_recovery_frames

    feature_values = [sample["features"] for sample in samples if sample["features"] is not None]
    feature_median = median(feature_values)
    features_pass = feature_median is not None and feature_median >= thresholds.min_median_features

    inlier_values = [sample["inliers"] for sample in samples if sample["inliers"] is not None]
    tracked_inliers = []
    for index in range(len(samples)):
        value = inlier_gate_value(samples, index)
        if value is not None:
            tracked_inliers.append(value)
    inlier_median = median(inlier_values)
    inliers_pass = bool(tracked_inliers) and all(value >= thresholds.min_inliers for value in tracked_inliers)
    distribution = {
        "min": json_number(min(inlier_values) if inlier_values else None),
        "median": json_number(inlier_median),
        "max": json_number(max(inlier_values) if inlier_values else None),
        "count": len(inlier_values),
        "below_threshold": sum(1 for value in inlier_values if value < thresholds.min_inliers),
    }

    metrics = {
        "lost_streak": {
            "value": streak,
            "threshold": thresholds.max_lost_streak,
            "pass": streak <= thresholds.max_lost_streak,
        },
        "recovery_frames": {
            "value": recovery_value,
            "threshold": thresholds.max_recovery_frames,
            "pass": recovery_pass,
            "episodes": episodes,
            "open_frames": open_frames,
        },
        "median_features": {
            "value": json_number(feature_median),
            "threshold": json_number(thresholds.min_median_features),
            "pass": features_pass,
        },
        "inliers": {
            "value": distribution,
            "threshold": json_number(thresholds.min_inliers),
            "pass": inliers_pass,
        },
        "odom_hz": odom_hz_metric(samples, thresholds.min_odom_hz),
    }
    return {
        "metrics": metrics,
        "pass": all(metrics[name]["pass"] for name in METRIC_NAMES),
    }


def odom_hz_metric(samples: list[dict], min_hz: float) -> dict:
    """Sim-time rate of ``/rtabmap/odom_info``, plus the wall-clock rate.

    ``ratio`` is ``wall_hz / sim_hz``, which is how many seconds of simulation
    time this topic advanced per wall second. The gate uses only the sim-time
    rate. Info-topic fallback samples are not included.
    """
    odom = [sample for sample in samples if sample.get("source") == "odom_info"]
    sim_hz = interval_rate([sample["stamp"] for sample in odom if sample.get("stamp") is not None])
    wall_times = [sample.get("wall_s") for sample in odom]
    wall_hz = None
    if wall_times and all(stamp is not None for stamp in wall_times):
        wall_hz = interval_rate([float(stamp) for stamp in wall_times])
    ratio = None
    if sim_hz not in (None, 0.0) and wall_hz is not None:
        ratio = wall_hz / sim_hz
    return {
        "value": json_number(sim_hz),
        "threshold": json_number(min_hz),
        "pass": sim_hz is not None and sim_hz >= min_hz,
        "wall_hz": json_number(wall_hz),
        "ratio": json_number(ratio),
    }


class TrackingLog:
    def __init__(self, min_inliers: float = DEFAULT_MIN_INLIERS, thresholds: VisionThresholds | None = None):
        if thresholds is None:
            thresholds = VisionThresholds(min_inliers=min_inliers)
        elif min_inliers != DEFAULT_MIN_INLIERS:
            thresholds = VisionThresholds(
                max_lost_streak=thresholds.max_lost_streak,
                max_recovery_frames=thresholds.max_recovery_frames,
                min_median_features=thresholds.min_median_features,
                min_inliers=min_inliers,
                min_odom_hz=thresholds.min_odom_hz,
            )
        self.thresholds = thresholds
        self.min_inliers = float(thresholds.min_inliers)
        self.loss_count = 0
        self.lost = False
        self.loss_started = None
        self.seen_loops = set()
        self.loop_count = 0
        self.last_stamp = None
        self.odom_info_seen = False
        self.samples: list[dict] = []

    def observe_odom(self, stamp, wall, lost, features, inliers, wall_s=None) -> list:
        """Record one ``/rtabmap/odom_info`` sample.

        ``lost``, ``features``, and ``inliers`` are the OdomInfo fields.
        ``stamp`` is the header stamp (simulation time). ``wall_s`` is a
        monotonic wall clock, used only to report the wall-clock rate.
        Later ``/rtabmap/info`` stats do not add another tracking sample.
        """
        self.odom_info_seen = True
        return self._record(
            stamp,
            wall,
            bool(lost),
            None if features is None else float(features),
            None if inliers is None else float(inliers),
            0,
            0,
            0,
            "odom_info.inliers" if inliers is not None else None,
            source="odom_info",
            wall_s=wall_s,
        )

    def update(self, stamp, wall, ref_id, loop_closure_id, proximity_id, stats) -> list:
        if self.odom_info_seen:
            return self._loop_events(stamp, wall, ref_id, loop_closure_id, proximity_id)
        key, inliers = pick_inliers(stats)
        lost_now = lost_from_stats(stats, inliers, self.min_inliers)
        return self._record(
            stamp,
            wall,
            lost_now,
            None,
            inliers,
            ref_id,
            loop_closure_id,
            proximity_id,
            key,
        )

    def _record(
        self,
        stamp,
        wall,
        lost_now,
        features,
        inliers,
        ref_id,
        loop_closure_id,
        proximity_id,
        inliers_key,
        source="info",
        wall_s=None,
    ) -> list:
        self.samples.append(
            {
                "lost": bool(lost_now),
                "features": features,
                "inliers": None if inliers is None else float(inliers),
                "stamp": None if stamp is None else float(stamp),
                "wall_s": None if wall_s is None else float(wall_s),
                "source": source,
            }
        )
        self.last_stamp = stamp
        events = [
            {
                "stamp": stamp,
                "wall_time": wall,
                "event": "frame",
                "ref_id": ref_id,
                "inliers": inliers,
                "inliers_key": inliers_key,
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
        events.extend(self._loop_events(stamp, wall, ref_id, loop_closure_id, proximity_id))
        return events

    def _loop_events(self, stamp, wall, ref_id, loop_closure_id, proximity_id) -> list:
        if not loop_closure_id or (ref_id, int(loop_closure_id)) in self.seen_loops:
            return []
        self.seen_loops.add((ref_id, int(loop_closure_id)))
        self.loop_count += 1
        return [
            {
                "stamp": stamp,
                "wall_time": wall,
                "event": "loop_closure",
                "loop_closure_id": int(loop_closure_id),
                "ref_id": ref_id,
                "loop_count": self.loop_count,
            }
        ]

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
        scored = score_samples(self.samples, self.thresholds)
        return {
            "stamp": self.last_stamp,
            "wall_time": wall,
            "event": "summary",
            "loss_count": self.loss_count,
            "loop_count": self.loop_count,
            "tracking": "lost" if self.lost else "ok",
            "metrics": scored["metrics"],
            "pass": scored["pass"],
        }


def write_jsonl(handle, event: dict) -> None:
    handle.write(json.dumps(event, sort_keys=True) + "\n")
    handle.flush()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Log RTAB-Map odometry health as JSONL.")
    parser.add_argument("--output", default="rtabmap_health.jsonl")
    parser.add_argument(
        "--min-inliers",
        type=float,
        default=None,
        help="Override VISION_MIN_INLIERS. Tracked frames below this fail the inlier gate.",
    )
    parser.add_argument("--fail-on-loss", action="store_true")
    return parser


def thresholds_from_args(argv: list[str] | None = None) -> tuple[VisionThresholds, argparse.Namespace]:
    """Apply CLI flags. Each flag overrides only its own field."""
    args = build_parser().parse_args(argv)
    thresholds = thresholds_from_env()
    if args.min_inliers is not None:
        thresholds = replace(thresholds, min_inliers=args.min_inliers)
    return thresholds, args


def main(argv: list[str] | None = None) -> int:
    load_repo_env()
    thresholds, args = thresholds_from_args(argv)
    fail = fail_on_loss_enabled(args.fail_on_loss)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rtabmap_msgs.msg import Info, OdomInfo

    rclpy.init()
    node = Node(
        "rtabmap_health_log",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    tracker = TrackingLog(thresholds=thresholds)
    info_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    output = open(args.output, "a", encoding="utf-8")

    def on_odom(msg: OdomInfo) -> None:
        events = tracker.observe_odom(
            stamp_seconds(msg.header.stamp),
            wall_now(),
            bool(msg.lost),
            int(msg.features),
            int(msg.inliers),
            wall_s=time.monotonic(),
        )
        for event in events:
            write_jsonl(output, event)

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

    # qos:=2 on the launch is best effort. A reliable subscriber would miss it.
    node.create_subscription(OdomInfo, "/rtabmap/odom_info", on_odom, qos_profile_sensor_data)
    node.create_subscription(Info, "/rtabmap/info", on_info, info_qos)
    node.get_logger().info(f"logging /rtabmap/odom_info and /rtabmap/info to {args.output}")
    summary = None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for event in tracker.finish(wall_now()):
            write_jsonl(output, event)
        summary = tracker.summary(wall_now())
        write_jsonl(output, summary)
        output.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fail and summary is not None and not summary["pass"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
