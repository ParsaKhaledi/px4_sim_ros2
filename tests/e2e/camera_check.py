"""Decide whether a camera frame is a real render or a blank image.

A software renderer that fails still publishes, often as an all-zero or
flat frame. Rate checks do not catch that. Variance does.
"""

from __future__ import annotations

import math
import struct
import time


def _channel_samples(data: bytes, encoding: str) -> list[float]:
    kind = (encoding or "").lower()
    if kind in {"rgb8", "bgr8"}:
        raw = data[::3]
    elif kind in {"rgba8", "bgra8"}:
        raw = data[::4]
    elif kind in {"mono8", "8uc1"}:
        raw = data
    elif kind in {"16uc1", "mono16"}:
        count = len(data) // 2
        raw = struct.unpack_from("<" + "H" * count, data) if count else ()
        return [float(value) for value in raw[:4000]]
    elif kind == "32fc1":
        count = len(data) // 4
        raw = struct.unpack_from("<" + "f" * count, data) if count else ()
        return [float(value) for value in raw[:4000] if math.isfinite(value)]
    else:
        raw = data
    return [float(value) for value in list(raw)[:4000]]


def pixel_stats(data: bytes, encoding: str) -> dict:
    values = _channel_samples(data, encoding)
    if not values:
        return {"mean": 0.0, "variance": 0.0, "count": 0}
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return {"mean": mean, "variance": variance, "count": len(values)}


def is_blank(stats: dict, min_variance: float) -> bool:
    if int(stats.get("count") or 0) <= 0:
        return True
    if float(stats["variance"]) < min_variance:
        return True
    if float(stats["mean"]) == 0.0 and float(stats["variance"]) == 0.0:
        return True
    return False


class CameraTracker:
    """Keep a short stats history per topic. The image bytes are not stored."""

    def __init__(self, min_variance: float = 1.0, topics: tuple[str, ...] | None = None):
        self.min_variance = min_variance
        self.required = topics or ("/camera/rgb/image_raw", "/camera/depth/image_raw")
        self.rows: dict[str, dict] = {}

    def add(self, topic: str, encoding: str, width: int, height: int, data: bytes, stamp_s: float) -> None:
        stats = pixel_stats(data, encoding)
        row = self.rows.get(topic)
        if row is None:
            row = {
                "frames": 0,
                "blank_frames": 0,
                "first_s": stamp_s,
                "last_s": stamp_s,
                "encoding": encoding,
                "width": width,
                "height": height,
                "mean": stats["mean"],
                "variance": stats["variance"],
            }
            self.rows[topic] = row
        row["frames"] += 1
        row["last_s"] = stamp_s
        row["encoding"] = encoding
        row["width"] = width
        row["height"] = height
        row["mean"] = stats["mean"]
        row["variance"] = stats["variance"]
        if is_blank(stats, self.min_variance):
            row["blank_frames"] += 1

    def report(self, now_s: float | None = None) -> dict:
        now = time.monotonic() if now_s is None else now_s
        topics = {}
        checks = []
        reason = None
        for topic in self.required:
            row = self.rows.get(topic)
            if row is None or row["frames"] == 0:
                topics[topic] = {"frames": 0, "rate_hz": 0.0, "blank": True}
                checks.append({
                    "name": f"camera:{topic}",
                    "passed": False,
                    "measured": 0.0,
                    "limit": self.min_variance,
                    "detail": "no frames",
                })
                reason = reason or "no_camera_frames"
                continue
            duration = max(row["last_s"] - row["first_s"], 0.0)
            rate = (row["frames"] / duration) if duration > 0.2 else float(row["frames"])
            latest = {
                "mean": row["mean"],
                "variance": row["variance"],
                "count": row["frames"],
            }
            blank = row["blank_frames"] * 2 >= row["frames"] or is_blank(latest, self.min_variance)
            topics[topic] = {
                "frames": row["frames"],
                "rate_hz": round(rate, 3),
                "mean": round(row["mean"], 4),
                "variance": round(row["variance"], 4),
                "blank_frames": row["blank_frames"],
                "blank": blank,
                "width": row["width"],
                "height": row["height"],
                "encoding": row["encoding"],
            }
            checks.append({
                "name": f"camera:{topic}",
                "passed": not blank,
                "measured": round(row["variance"], 4),
                "limit": self.min_variance,
                "detail": f"rate {rate:.2f} Hz, mean {row['mean']:.2f}",
            })
            if blank and reason is None:
                reason = "blank_camera"
        # `now` is available for callers that want an open-ended rate window.
        del now
        return {
            "passed": reason is None,
            "reason": reason,
            "min_variance": self.min_variance,
            "topics": topics,
            "checks": checks,
        }


def rtf_summary(samples: list[float]) -> dict | None:
    if not samples:
        return None
    return {
        "mean": round(sum(samples) / len(samples), 4),
        "min": round(min(samples), 4),
        "max": round(max(samples), 4),
        "samples": len(samples),
    }
