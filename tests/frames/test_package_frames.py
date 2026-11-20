"""Run the package conversion tests from ``pytest tests/frames``.

``px4_control/test/test_frames.py`` and ``test_vision_and_modes.py`` already
cover the ENU/NED maps. The cases that belong with them (tilted attitudes,
covariance rotation, frame-id rejection) live in those files. This module
collects them so the repo-root entry point runs the same functions.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_PACKAGE = _ROOT / "ros2_ws" / "src" / "px4_control" / "test"


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, _PACKAGE / filename)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _export(module) -> None:
    for name, obj in vars(module).items():
        if name.startswith("test_"):
            globals()[name] = obj


_export(_load("px4_control_test_frames", "test_frames.py"))
_export(_load("px4_control_test_vision_and_modes", "test_vision_and_modes.py"))
