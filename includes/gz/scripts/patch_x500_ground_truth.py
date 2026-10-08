#!/usr/bin/env python3
"""Insert a Gazebo OdometryPublisher into the PX4 x500_depth model.

The plugin publishes the model's true world pose (Gazebo ENU, Z up) on
``/ground_truth/odom`` at 50 Hz, and the same pose with covariance on
``/ground_truth/odom_with_covariance``. The covariance topic is set
explicitly: the plugin default is ``/model/<model_name>/odometry_with_covariance``,
which is the topic PX4 v1.17 GZBridge subscribes to and feeds to EKF2 as
external vision. The edit is idempotent and only runs when the model file
is present (inside the PX4 SITL image).
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

COVARIANCE_TOPIC = "/ground_truth/odom_with_covariance"

PLUGIN = """\
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link_gt</robot_base_frame>
      <odom_publish_frequency>50</odom_publish_frequency>
      <odom_topic>/ground_truth/odom</odom_topic>
      <odom_covariance_topic>/ground_truth/odom_with_covariance</odom_covariance_topic>
      <tf_topic>/ground_truth/tf</tf_topic>
      <dimensions>3</dimensions>
    </plugin>
"""

MARKER = "gz::sim::systems::OdometryPublisher"
_COVARIANCE_ELEMENT = re.compile(
    r"<odom_covariance_topic>\s*.*?\s*</odom_covariance_topic>",
    re.DOTALL,
)


def candidate_models() -> list[Path]:
    """Return x500_depth model.sdf paths that may exist on this machine."""
    raw: list[Path] = []
    env_models = os.environ.get("PX4_GZ_MODELS", "")
    if env_models:
        raw.append(Path(env_models) / "x500_depth" / "model.sdf")
    home = Path.home()
    raw.append(
        home / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models" / "x500_depth" / "model.sdf"
    )
    raw.append(
        Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf")
    )
    found: list[Path] = []
    seen: set[Path] = set()
    for path in raw:
        key = path.resolve()
        if key in seen:
            continue
        seen.add(key)
        found.append(path)
    return found


def _plugin_blocks(model_xml: str) -> list[tuple[int, int]]:
    """Byte spans of each plugin element that contains the ground-truth marker."""
    spans: list[tuple[int, int]] = []
    cursor = 0
    while True:
        marker_at = model_xml.find(MARKER, cursor)
        if marker_at < 0:
            return spans
        open_at = model_xml.rfind("<plugin", 0, marker_at)
        close_at = model_xml.find("</plugin>", marker_at)
        if open_at < 0 or close_at < 0:
            return spans
        close_at += len("</plugin>")
        spans.append((open_at, close_at))
        cursor = close_at


def _ensure_covariance(body: str) -> str:
    """Set odom_covariance_topic inside one plugin body."""
    element = f"<odom_covariance_topic>{COVARIANCE_TOPIC}</odom_covariance_topic>"
    if _COVARIANCE_ELEMENT.search(body):
        return _COVARIANCE_ELEMENT.sub(element, body)
    odom_topic = re.search(r"<odom_topic>\s*[^<]*\s*</odom_topic>", body)
    if odom_topic:
        return body[: odom_topic.end()] + "\n      " + element + body[odom_topic.end() :]
    stripped = body.rstrip()
    trailer = "\n" if body.endswith("\n") else "\n"
    return stripped + "\n      " + element + trailer


def _repair_plugin(plugin_xml: str) -> str:
    open_end = plugin_xml.find(">") + 1
    close = plugin_xml.rfind("</plugin>")
    head, body, tail = plugin_xml[:open_end], plugin_xml[open_end:close], plugin_xml[close:]
    body = body.replace(
        "<robot_base_frame>base_link</robot_base_frame>",
        "<robot_base_frame>base_link_gt</robot_base_frame>",
    )
    return head + _ensure_covariance(body) + tail


def covariance_topic_problems(model_xml: str) -> list[str]:
    """Reasons an OdometryPublisher would publish on PX4's vision topic.

    A plugin with no ``odom_covariance_topic`` uses the default
    ``/model/<model_name>/odometry_with_covariance``. A topic that starts
    with ``/model/`` is that same default, or a rename of it.
    """
    problems: list[str] = []
    for open_at, close_at in _plugin_blocks(model_xml):
        block = model_xml[open_at:close_at]
        match = _COVARIANCE_ELEMENT.search(block)
        if match is None:
            problems.append("OdometryPublisher has no odom_covariance_topic")
            continue
        inner = re.search(
            r"<odom_covariance_topic>\s*(.*?)\s*</odom_covariance_topic>",
            match.group(0),
            re.DOTALL,
        )
        topic = re.sub(r"\s+", "", inner.group(1) if inner else "")
        if topic.startswith("/model/"):
            problems.append(f"odom_covariance_topic {topic} starts with /model/")
    return problems


def repo_model_sdfs() -> list[Path]:
    """Every ``model.sdf`` under ``includes/gz/models``."""
    root = Path(__file__).resolve().parents[1] / "models"
    if not root.is_dir():
        return []
    return sorted(root.rglob("model.sdf"))


def px4_x500_model_sdfs() -> list[Path]:
    """PX4 ``x500*`` model files, when that tree is on this machine."""
    roots: list[Path] = []
    env_models = os.environ.get("PX4_GZ_MODELS", "")
    if env_models:
        roots.append(Path(env_models))
    roots.append(Path.home() / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models")
    roots.append(Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models"))
    found: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        if not root.is_dir():
            continue
        for path in sorted(root.glob("x500*/model.sdf")):
            key = path.resolve()
            if key in seen:
                continue
            seen.add(key)
            found.append(path)
    return found


def watched_model_sdfs() -> list[Path]:
    """Repo models plus PX4 x500 models that exist in this environment."""
    return repo_model_sdfs() + px4_x500_model_sdfs()


def patch_text(model_xml: str) -> str:
    """Return model XML with the ground-truth plugin inserted once.

    When the plugin is already present, ``robot_base_frame`` ``base_link``
    is rewritten to ``base_link_gt`` and ``odom_covariance_topic`` is set to
    ``/ground_truth/odom_with_covariance`` (inserted when missing, replaced
    when it points at ``/model/.../odometry_with_covariance``).
    """
    if MARKER in model_xml:
        updated = model_xml
        for open_at, close_at in reversed(_plugin_blocks(model_xml)):
            updated = updated[:open_at] + _repair_plugin(updated[open_at:close_at]) + updated[close_at:]
        return updated
    close = model_xml.rfind("</model>")
    if close < 0:
        raise ValueError("model.sdf has no </model> element")
    return model_xml[:close] + PLUGIN + model_xml[close:]


def patch_file(path: Path) -> bool:
    """Patch one model file. Return True when the file changed."""
    original = path.read_text(encoding="utf-8")
    updated = patch_text(original)
    if updated == original:
        print(f"ground truth plugin already present: {path}")
        return False
    path.write_text(updated, encoding="utf-8")
    print(f"added ground truth OdometryPublisher to {path}")
    return True


def main() -> int:
    patched = False
    for path in candidate_models():
        if path.is_file():
            patch_file(path)
            patched = True
    if not patched:
        print(
            "x500_depth model.sdf not found; ground-truth plugin was not applied. "
            "This is expected outside the PX4 SITL container.",
            file=sys.stderr,
        )
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
