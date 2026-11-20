"""Import the real bridge and the static robot description without ROS."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for extra in (
    ROOT / "ros2_ws" / "src" / "px4_control",
    ROOT / "tests" / "e2e",
    ROOT / "tests" / "frames",
    ROOT / "includes" / "gz" / "oakd_s2",
):
    text = str(extra)
    if text not in sys.path:
        sys.path.insert(0, text)
