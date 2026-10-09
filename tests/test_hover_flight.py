"""Flight level: grade the one headless hover with tests/e2e.

This does not start the simulator, and it does not grade an out-and-back.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from grading import grade_hover, load_hover_track  # noqa: E402


@pytest.mark.flight
def test_headless_hover():
    try:
        samples = load_hover_track()
    except ValueError as exc:
        pytest.fail(str(exc))
    result = grade_hover(samples)
    assert result["passed"], result["checks"]
    names = {item["name"] for item in result["checks"]}
    assert names.isdisjoint({"leg1", "leg2", "yaw_settle", "return_to_start"})
