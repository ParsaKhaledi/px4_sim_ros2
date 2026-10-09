#!/usr/bin/env python3
"""Per-service health checker.

Measures topic rates with rclpy, writes one JSON line per check under
logs/health/<service>.jsonl, and prints a short summary for Docker.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

SERVICE_FILES = {
    "PX4": "px4.yaml",
    "StatePublisher": "state_publisher.yaml",
    "Rtabmap": "rtabmap.yaml",
    "NAV2": "nav2.yaml",
    "Qground": "qground.yaml",
    "Nav2_Rviz": "nav2_rviz.yaml",
}

ENV_DEFAULTS = {
    "CameraType": "rgbd",
    "World": "default",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit(
            "PyYAML is required (python3-yaml). Install it in the image or the host venv."
        ) from exc
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"{path} must be a mapping")
    return data


def topic_matches_env(spec: dict, env: dict) -> bool:
    cond = spec.get("when_env") or {}
    for key, expected in cond.items():
        actual = env.get(key)
        if actual is None or actual == "":
            actual = ENV_DEFAULTS.get(key, "")
        if str(actual).lower() != str(expected).lower():
            return False
    return True


def topic_version(name: str) -> int:
    """PX4 appends _vN when the uORB message version is non-zero."""
    match = re.search(r"_v(\d+)$", name)
    return int(match.group(1)) if match else 0


def choose_topic(candidates, graph_topics, publisher_counts) -> str | None:
    """Prefer a live publisher, then the versioned topic name.

    A subscriber can put the unversioned name in the graph with no
    publisher. That name must not win over vehicle_status_v1.
    """
    best = None
    best_key = None
    for index, name in enumerate(candidates):
        publishers = publisher_counts.get(name, 0)
        if publishers <= 0 and name not in graph_topics:
            continue
        key = (publishers > 0, topic_version(name), -index)
        if best_key is None or key > best_key:
            best = name
            best_key = key
    return best


def effective_min_rate(spec: dict, env: dict | None = None, clock_hz: float | None = None) -> float:
    """Software rendering cannot hold the 5 Hz camera floor.

    A topic with ``model_rate_hz`` and ``step_rate_hz`` is published on
    sim time. Its wall-clock rate tracks ``/clock``, which is one message
    per physics step. The floor is

        clock_hz * (model_rate_hz / step_rate_hz) * rate_margin

    so a slower world is not held to a fixed wall-clock number. With no
    clock sample the floor stays positive, so 0 Hz still fails.
    """
    source = os.environ if env is None else env
    if spec.get("group") == "camera":
        override = source.get("HEALTH_CAMERA_MIN_HZ")
        if override not in (None, ""):
            return float(override)
    model_rate = spec.get("model_rate_hz")
    step_rate = spec.get("step_rate_hz")
    if model_rate not in (None, "") and step_rate not in (None, ""):
        margin = float(spec.get("rate_margin", 0.5))
        if clock_hz and float(clock_hz) > 0 and float(step_rate) > 0:
            return float(clock_hz) * (float(model_rate) / float(step_rate)) * margin
        return 1.0
    return float(spec.get("min_rate_hz", 0))


def judge_rate(publishers, count, window_s, min_rate, skip_if_absent, optional):
    rate = (float(count) / float(window_s)) if window_s else 0.0
    if publishers <= 0:
        if skip_if_absent or optional:
            return "skip", rate, "topic absent"
        return "fail", rate, "no publisher"
    if rate + 1e-6 < float(min_rate):
        return "fail", rate, f"rate {rate:.2f} Hz below {min_rate}"
    return "ok", rate, ""


def process_alive(pattern: str) -> bool:
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return False
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes().replace(b"\x00", b" ")
        except OSError:
            continue
        text = raw.decode("utf-8", errors="ignore")
        if pattern in text:
            return True
    return False


def append_jsonl(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")


def default_log_dir() -> Path:
    env = os.environ.get("HEALTH_LOG_DIR")
    if env:
        return Path(env)
    container = Path("/home/px4/volume/logs/health")
    if container.parent.exists() or Path("/home/px4/volume").exists():
        return container
    return Path("logs/health")


def qos_profile(name: str):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    if name == "transient_local":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    if name == "reliable":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def import_message(type_name: str):
    if "/msg/" not in type_name:
        return None
    package, name = type_name.split("/msg/", 1)
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, name)


def read_gz_rtf(world: str, timeout_s: float = 3.0):
    topic = f"/world/{world}/stats"
    try:
        proc = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    text = (proc.stdout or "") + "\n" + (proc.stderr or "")
    for line in text.splitlines():
        if "real_time_factor" in line and ":" in line:
            raw = line.split(":", 1)[1].strip()
            try:
                return float(raw)
            except ValueError:
                continue
    return None


def snapshot_message(msg) -> dict:
    fields = {}
    for attr in (
        "arming_state",
        "nav_state",
        "failsafe",
        "pre_flight_checks_pass",
        "gcs_connection_lost",
    ):
        if hasattr(msg, attr):
            value = getattr(msg, attr)
            if isinstance(value, (int, float, bool, str)):
                fields[attr] = value
    return fields


class HealthRunner:
    def __init__(self, service: str, config: dict, group: str | None, log_path: Path):
        self.service = service
        self.config = config
        self.group = group
        self.log_path = log_path
        self.results: list[dict] = []
        self.env = dict(os.environ)

    def record(self, **fields) -> dict:
        row = {
            "ts": utc_now(),
            "service": self.service,
            "duration_ms": fields.pop("duration_ms", 0),
        }
        row.update(fields)
        self.results.append(row)
        append_jsonl(self.log_path, row)
        return row

    def selected(self, spec: dict) -> bool:
        if self.group and spec.get("group") != self.group:
            return False
        return topic_matches_env(spec, self.env)

    def run(self) -> int:
        started = time.monotonic()
        self._processes()
        needs_ros = any(
            self.selected(spec)
            for key in ("topics", "nodes", "lifecycle", "tf_frames")
            for spec in self.config.get(key, [])
        ) or (
            self.config.get("gz_rtf", {}).get("enabled")
            and (not self.group or self.group == "gz")
        )
        if needs_ros or self.config.get("topics") or self.config.get("nodes"):
            self._ros_checks()
        if self.config.get("gz_rtf", {}).get("enabled") and (not self.group or self.group == "gz"):
            self._gz_rtf()
        fails = [row for row in self.results if row["status"] == "fail"]
        oks = [row for row in self.results if row["status"] == "ok"]
        skips = [row for row in self.results if row["status"] == "skip"]
        elapsed_ms = int((time.monotonic() - started) * 1000)
        print(
            f"{self.service} status={'fail' if fails else 'ok'} "
            f"ok={len(oks)} fail={len(fails)} skip={len(skips)} duration_ms={elapsed_ms}"
        )
        for row in fails:
            reason = row.get("reason") or ""
            print(
                f"FAIL {row.get('check')} {row.get('target', '')} {reason}".rstrip()
            )
        return 1 if fails else 0

    def _processes(self) -> None:
        for spec in self.config.get("processes", []):
            if not self.selected(spec):
                continue
            started = time.monotonic()
            alive = process_alive(spec["pattern"])
            optional = bool(spec.get("optional"))
            if alive:
                status, reason = "ok", ""
            elif optional:
                status, reason = "skip", "process absent"
            else:
                status, reason = "fail", "process not running"
            self.record(
                check="process",
                target=spec.get("name", spec["pattern"]),
                status=status,
                reason=reason,
                duration_ms=int((time.monotonic() - started) * 1000),
            )

    def _gz_rtf(self) -> None:
        started = time.monotonic()
        world = self.env.get("World") or ENV_DEFAULTS["World"]
        factor = read_gz_rtf(world)
        if factor is None:
            status, reason = "skip", f"no real_time_factor on /world/{world}/stats"
        else:
            status, reason = "ok", ""
        self.record(
            check="gz_rtf",
            target=f"/world/{world}/stats",
            status=status,
            real_time_factor=factor,
            reason=reason,
            duration_ms=int((time.monotonic() - started) * 1000),
        )

    def _ros_checks(self) -> None:
        try:
            import rclpy
        except ImportError as exc:
            self.record(
                check="rclpy",
                target="import",
                status="fail",
                reason=str(exc),
            )
            return
        rclpy.init(args=None)
        node = rclpy.create_node("px4_healthcheck", enable_rosout=False)
        try:
            self._wait_graph(node, rclpy)
            graph = {name: types for name, types in node.get_topic_names_and_types()}
            counts = {name: node.count_publishers(name) for name in graph}
            resolved = self._resolve_topics(graph, counts)
            measurements = self._measure(node, rclpy, resolved)
            self._judge_topics(resolved, measurements)
            self._nodes(node)
            self._lifecycle(node, rclpy)
            self._tf(node, rclpy, measurements)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _wait_graph(self, node, rclpy) -> None:
        deadline = time.monotonic() + float(self.config.get("discovery_s", 2.0))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

    def _resolve_topics(self, graph, counts) -> list[dict]:
        resolved = []
        for spec in self.config.get("topics", []):
            if not self.selected(spec):
                continue
            if spec.get("auto_detect"):
                chosen = choose_topic(spec["auto_detect"], set(graph), counts)
                target = chosen or spec["auto_detect"][0]
                absent = chosen is None
            else:
                target = spec["topic"]
                absent = target not in graph and counts.get(target, 0) <= 0
            publishers = counts.get(target, 0) if not absent else 0
            type_name = None
            types = graph.get(target) or []
            if types:
                type_name = types[0]
            resolved.append(
                {
                    "spec": spec,
                    "target": target,
                    "publishers": publishers,
                    "type_name": type_name,
                    "absent": absent and publishers <= 0,
                }
            )
        return resolved

    def _measure(self, node, rclpy, resolved) -> dict:
        window_s = float(self.config.get("window_s", 2.0))
        state = {}
        subscriptions = []
        for item in resolved:
            if item["publishers"] <= 0:
                continue
            type_name = item["type_name"]
            if not type_name:
                continue
            try:
                msg_type = import_message(type_name)
            except (ImportError, AttributeError):
                item["import_error"] = type_name
                continue
            target = item["target"]
            bucket = {"count": 0, "last_mono": None, "last_msg": None}

            def _callback(msg, bucket=bucket):
                bucket["count"] += 1
                bucket["last_mono"] = time.monotonic()
                bucket["last_msg"] = msg

            qos_name = item["spec"].get("qos", "sensor")
            subscriptions.append(
                node.create_subscription(msg_type, target, _callback, qos_profile(qos_name))
            )
            state[target] = bucket

        if state:
            deadline = time.monotonic() + window_s
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        return state

    def _judge_topics(self, resolved, measurements) -> None:
        window_s = float(self.config.get("window_s", 2.0))
        now = time.monotonic()
        for item in resolved:
            spec = item["spec"]
            started = time.monotonic()
            target = item["target"]
            bucket = measurements.get(target)
            count = bucket["count"] if bucket else 0
            clock_bucket = measurements.get("/clock")
            clock_hz = (float(clock_bucket["count"]) / window_s) if clock_bucket and window_s else 0.0
            min_rate = effective_min_rate(spec, self.env, clock_hz)
            status, rate, reason = judge_rate(
                publishers=item["publishers"],
                count=count,
                window_s=window_s,
                min_rate=min_rate,
                skip_if_absent=bool(spec.get("skip_if_absent")),
                optional=bool(spec.get("optional")),
            )
            if item.get("import_error") and item["publishers"] > 0 and count == 0:
                status = "fail"
                reason = f"could not import {item['import_error']}"
            elif status == "fail" and spec.get("step_rate_hz") and clock_hz > 0:
                reason = (
                    f"{reason} "
                    f"(clock {clock_hz:.1f} Hz, "
                    f"model {spec.get('model_rate_hz')} Hz / step {spec.get('step_rate_hz')} Hz)"
                )
            age = None
            if bucket and bucket["last_mono"] is not None:
                age = round(now - bucket["last_mono"], 4)
            extra = {}
            if bucket and bucket["last_msg"] is not None and spec.get("capture_fields"):
                extra = snapshot_message(bucket["last_msg"])
            self.record(
                check="topic_rate",
                target=target,
                status=status,
                publishers=item["publishers"],
                rate_hz=round(rate, 3),
                min_rate_hz=min_rate,
                last_msg_age_s=age,
                reason=reason,
                fields=extra or None,
                duration_ms=int((time.monotonic() - started) * 1000),
            )

    def _nodes(self, node) -> None:
        names = set(node.get_node_names())
        for spec in self.config.get("nodes", []):
            if not self.selected(spec):
                continue
            present = spec["name"] in names
            optional = bool(spec.get("optional"))
            if present:
                status, reason = "ok", ""
            elif optional:
                status, reason = "skip", "node absent"
            else:
                status, reason = "fail", "node not in graph"
            self.record(
                check="node",
                target=spec["name"],
                status=status,
                reason=reason,
            )

    def _lifecycle(self, node, rclpy) -> None:
        specs = [spec for spec in self.config.get("lifecycle", []) if self.selected(spec)]
        if not specs:
            return
        try:
            from lifecycle_msgs.srv import GetState
        except ImportError as exc:
            self.record(check="lifecycle", target="lifecycle_msgs", status="fail", reason=str(exc))
            return
        for spec in specs:
            name = spec["name"]
            started = time.monotonic()
            client = node.create_client(GetState, f"{name}/get_state")
            if not client.wait_for_service(timeout_sec=float(spec.get("timeout_s", 2.0))):
                status = "skip" if spec.get("optional") else "fail"
                self.record(
                    check="lifecycle",
                    target=name,
                    status=status,
                    reason="get_state service unavailable",
                    duration_ms=int((time.monotonic() - started) * 1000),
                )
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if not future.done() or future.result() is None:
                self.record(
                    check="lifecycle",
                    target=name,
                    status="fail",
                    reason="get_state timed out",
                    duration_ms=int((time.monotonic() - started) * 1000),
                )
                continue
            label = future.result().current_state.label
            wanted = spec.get("state", "active")
            status = "ok" if label == wanted else "fail"
            self.record(
                check="lifecycle",
                target=name,
                status=status,
                lifecycle_state=label,
                reason="" if status == "ok" else f"state is {label}, want {wanted}",
                duration_ms=int((time.monotonic() - started) * 1000),
            )

    def _tf(self, node, rclpy, measurements) -> None:
        frames_needed = [
            spec for spec in self.config.get("tf_frames", []) if self.selected(spec)
        ]
        if not frames_needed:
            return
        try:
            from tf2_msgs.msg import TFMessage
        except ImportError as exc:
            self.record(check="tf", target="/tf_static", status="fail", reason=str(exc))
            return
        seen: set[str] = set()

        def _on_tf(msg):
            for transform in msg.transforms:
                seen.add(transform.child_frame_id)

        sub = node.create_subscription(
            TFMessage, "/tf_static", _on_tf, qos_profile("transient_local")
        )
        deadline = time.monotonic() + float(self.config.get("window_s", 2.0))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        for spec in frames_needed:
            frame = spec["frame"]
            present = frame in seen
            optional = bool(spec.get("optional"))
            if present:
                status, reason = "ok", ""
            elif optional:
                status, reason = "skip", "frame not in /tf_static"
            else:
                status, reason = "fail", "frame not in /tf_static"
            self.record(check="tf", target=frame, status=status, reason=reason, frames=sorted(seen))


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS 2 service health check")
    parser.add_argument("--service", required=True, choices=sorted(SERVICE_FILES))
    parser.add_argument("--config-dir", default=None)
    parser.add_argument("--group", default=None, help="Only run checks tagged with this group")
    parser.add_argument("--log-dir", default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    config_dir = Path(args.config_dir) if args.config_dir else Path(__file__).resolve().parent / "checks"
    config_path = config_dir / SERVICE_FILES[args.service]
    if not config_path.is_file():
        print(f"missing health config {config_path}", file=sys.stderr)
        return 1
    config = load_yaml(config_path)
    log_dir = Path(args.log_dir) if args.log_dir else default_log_dir()
    log_name = config.get("log_name") or args.service.lower()
    runner = HealthRunner(args.service, config, args.group, log_dir / f"{log_name}.jsonl")
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
