#!/usr/bin/env python3
"""Keep the stable repo ``x500_depth`` model’s ground-truth plugin up to date.

The committed model is ``includes/gz/models/x500_depth/model.sdf``. It already
contains ``OdometryPublisher`` so SITL start does not patch PX4 at runtime.
``gz_modifications.bash`` copies that folder into the PX4 model tree.

Run this script when you change the stock layout or the plugin block. It
rewrites only the repo model. It does not edit ``~/PX4-Autopilot`` at start.

The plugin publishes the model’s true world pose (Gazebo ENU, Z up) on
``/ground_truth/odom`` at 50 Hz, and the same pose with covariance on
``/ground_truth/odom_with_covariance``. The covariance topic is set
explicitly: the plugin default is ``/model/<model_name>/odometry_with_covariance``,
which PX4 v1.17 GZBridge feeds to EKF2 as external vision.
"""

from __future__ import annotations

import argparse
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

STOCK_X500_DEPTH = """\
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_depth'>
    <include merge='true'>
      <uri>x500</uri>
    </include>
    <include merge='true'>
      <uri>model://OakD-Lite</uri>
      <pose>.12 .03 .242 0 0 0</pose>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
    </joint>
  </model>
</sdf>
"""


def repo_models_root() -> Path:
    """``includes/gz/models`` next to this scripts package."""
    return Path(__file__).resolve().parents[1] / "models"


def repo_x500_depth_sdf() -> Path:
    """Stable overlay that ``gz_modifications.bash`` copies into PX4."""
    return repo_models_root() / "x500_depth" / "model.sdf"


def candidate_models() -> list[Path]:
    """Return the repo ``x500_depth`` path first, then optional PX4 lookups.

    Tests still call this name. Runtime generation targets the repo path only.
    """
    raw: list[Path] = [repo_x500_depth_sdf()]
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
        try:
            key = path.resolve()
        except OSError:
            key = path
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
    root = repo_models_root()
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


def update_repo_model(*, source: Path | None = None, reset_stock: bool = False) -> bool:
    """Write the stable repo ``x500_depth`` SDF with the ground-truth plugin.

    Parameters
    ----------
    source:
        Optional stock ``model.sdf`` to start from (for example a fresh PX4
        copy). When omitted, the existing repo file is used, or the built-in
        stock template if the file is missing.
    reset_stock:
        Ignore the current repo file and rebuild from ``STOCK_X500_DEPTH``
        (or ``source`` when given).
    """
    dest = repo_x500_depth_sdf()
    dest.parent.mkdir(parents=True, exist_ok=True)
    if source is not None:
        base = source.read_text(encoding="utf-8")
    elif reset_stock or not dest.is_file():
        base = STOCK_X500_DEPTH
    else:
        base = dest.read_text(encoding="utf-8")
    updated = patch_text(base)
    if dest.is_file() and dest.read_text(encoding="utf-8") == updated:
        print(f"repo x500_depth already up to date: {dest}")
        return False
    dest.write_text(updated, encoding="utf-8")
    print(f"updated stable repo model: {dest}")
    return True


def main(argv: list[str] | None = None) -> int:
    """Update ``includes/gz/models/x500_depth/model.sdf`` only."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--from",
        dest="source",
        type=Path,
        help="Stock model.sdf to start from (default: current repo file or built-in stock)",
    )
    parser.add_argument(
        "--reset-stock",
        action="store_true",
        help="Rebuild from the built-in stock x500_depth layout (plus the GT plugin)",
    )
    args = parser.parse_args(argv)
    if args.source is not None and not args.source.is_file():
        print(f"source model not found: {args.source}", file=sys.stderr)
        return 1
    update_repo_model(source=args.source, reset_stock=args.reset_stock)
    problems = covariance_topic_problems(repo_x500_depth_sdf().read_text(encoding="utf-8"))
    if problems:
        print("covariance topic problems:", *problems, sep="\n  ", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
