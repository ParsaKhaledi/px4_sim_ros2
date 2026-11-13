#!/usr/bin/env python3
"""Ground-truth OdometryPublisher for PX4 Gazebo x500 airframes.

Two jobs:

1. **Runtime (default).** Flight CI sets ``PX4_GZ_MODEL=x500``. The camera
   airframe is ``x500_depth``. At SITL start this script inserts the plugin
   into whichever of those models exist in the PX4 tree, so
   ``/ground_truth/odom`` is published by the model that actually loads. A
   bridge entry alone advertises the ROS topic and the health check then
   sees 0 Hz.

2. **Repo bake (``--repo``).** Rewrites only
   ``includes/gz/models/x500_depth/model.sdf``. ``gz_modifications.bash``
   copies that overlay into PX4, so camera flights already have the plugin
   before this script runs (idempotent no-op).

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


def _unique_paths(raw: list[Path]) -> list[Path]:
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


def model_sdf_candidates(model_name: str) -> list[Path]:
    """Return ``model.sdf`` paths for one PX4 gz airframe."""
    raw: list[Path] = []
    env_models = os.environ.get("PX4_GZ_MODELS", "")
    if env_models:
        raw.append(Path(env_models) / model_name / "model.sdf")
    home = Path.home()
    raw.append(
        home / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models" / model_name / "model.sdf"
    )
    raw.append(
        Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models") / model_name / "model.sdf"
    )
    return _unique_paths(raw)


def airframe_name(environ: dict[str, str] | None = None) -> str:
    """Model directory PX4 will spawn. ``gz_`` and a trailing instance are stripped."""
    env = os.environ if environ is None else environ
    raw = env.get("PX4_GZ_MODEL", "").strip()
    if not raw:
        raw = env.get("PX4_SIM_MODEL", "").strip()
        if raw.startswith("gz_"):
            raw = raw[3:]
    if re.fullmatch(r".+_\d+", raw):
        raw = raw.rsplit("_", 1)[0]
    return raw


def ground_truth_model_names(environ: dict[str, str] | None = None) -> list[str]:
    """Camera airframe plus the airframe this process will spawn.

    ``x500_depth`` stays in the list so a camera flight still gets the
    plugin when ``PX4_GZ_MODEL`` is unset. Flight CI sets ``x500``.
    """
    names = ["x500_depth"]
    selected = airframe_name(environ)
    if selected and selected not in names:
        names.append(selected)
    return names


def candidate_models() -> list[Path]:
    """Return the repo ``x500_depth`` path first, then optional PX4 lookups.

    Tests still call this name. Runtime generation targets the spawned
    airframes via ``ground_truth_models``.
    """
    raw: list[Path] = [repo_x500_depth_sdf()]
    raw.extend(model_sdf_candidates("x500_depth"))
    return _unique_paths(raw)


def ground_truth_models(environ: dict[str, str] | None = None) -> list[Path]:
    """model.sdf paths that should carry the ground-truth plugin at SITL start."""
    raw: list[Path] = []
    for name in ground_truth_model_names(environ):
        raw.extend(model_sdf_candidates(name))
    return _unique_paths(raw)


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


def patch_runtime_models() -> int:
    """Patch the spawned airframe and x500_depth in the PX4 tree. Missing files are skipped."""
    names = ", ".join(ground_truth_model_names())
    patched = False
    for path in ground_truth_models():
        if path.is_file():
            patch_file(path)
            patched = True
    if not patched:
        print(
            f"model.sdf not found for {names}; ground-truth plugin was not applied. "
            "This is expected outside the PX4 SITL container.",
            file=sys.stderr,
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    """Runtime-patch PX4 models by default; ``--repo`` refreshes the baked overlay."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        action="store_true",
        help="Rewrite includes/gz/models/x500_depth/model.sdf only (no PX4 tree edit)",
    )
    parser.add_argument(
        "--from",
        dest="source",
        type=Path,
        help="Stock model.sdf to start from when using --repo",
    )
    parser.add_argument(
        "--reset-stock",
        action="store_true",
        help="With --repo, rebuild from the built-in stock x500_depth layout",
    )
    args = parser.parse_args(argv)
    if args.repo or args.source is not None or args.reset_stock:
        if args.source is not None and not args.source.is_file():
            print(f"source model not found: {args.source}", file=sys.stderr)
            return 1
        update_repo_model(source=args.source, reset_stock=args.reset_stock)
        problems = covariance_topic_problems(repo_x500_depth_sdf().read_text(encoding="utf-8"))
        if problems:
            print("covariance topic problems:", *problems, sep="\n  ", file=sys.stderr)
            return 1
        return 0
    return patch_runtime_models()


if __name__ == "__main__":
    raise SystemExit(main())
