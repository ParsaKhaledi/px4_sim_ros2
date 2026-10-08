#!/usr/bin/env python3
"""Insert a Gazebo OdometryPublisher into the PX4 x500_depth model.

The plugin publishes the model's true world pose (Gazebo ENU, Z up) on
``/ground_truth/odom`` at 50 Hz. The edit is idempotent and only runs when
the model file is present (inside the PX4 SITL image).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

PLUGIN = """\
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>50</odom_publish_frequency>
      <odom_topic>/ground_truth/odom</odom_topic>
      <tf_topic>/ground_truth/tf</tf_topic>
      <dimensions>3</dimensions>
    </plugin>
"""

MARKER = "gz::sim::systems::OdometryPublisher"


def candidate_models() -> list[Path]:
    """Return x500_depth model.sdf paths that may exist on this machine."""
    found: list[Path] = []
    env_models = os.environ.get("PX4_GZ_MODELS", "")
    if env_models:
        found.append(Path(env_models) / "x500_depth" / "model.sdf")
    home = Path.home()
    found.append(
        home / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models" / "x500_depth" / "model.sdf"
    )
    found.append(
        Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf")
    )
    return found


def patch_text(model_xml: str) -> str:
    """Return model XML with the ground-truth plugin inserted once."""
    if MARKER in model_xml:
        return model_xml
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
