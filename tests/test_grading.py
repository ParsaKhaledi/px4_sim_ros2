import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from grading import grade_mission, load_thresholds  # noqa: E402


def _sample(phase, x, y, z, yaw):
    return {"phase": phase, "x": x, "y": y, "z": z, "yaw": yaw}


def _perfect(leg=0.3):
    rows = [_sample("start", 0.0, 0.0, 0.1, 0.0)]
    for _ in range(5):
        rows.append(_sample("hover", 0.01, 0.0, 2.1, 0.0))
    rows.append(_sample("leg1", 0.01 + leg, 0.0, 2.1, 0.0))
    rows.append(_sample("yaw", 0.01 + leg, 0.0, 2.1, 179.0))
    rows.append(_sample("yaw", 0.01 + leg, 0.0, 2.1, 180.0))
    rows.append(_sample("leg2", 0.01, 0.0, 2.1, 180.0))
    rows.append(_sample("land", 0.01, 0.0, 0.1, 180.0))
    return rows


def test_perfect_mission_passes():
    result = grade_mission(_perfect())
    assert result["passed"], result["checks"]


def test_hover_drift_fails():
    rows = _perfect()
    rows.append(_sample("hover", 0.2, 0.0, 2.1, 0.0))
    result = grade_mission(rows)
    drift = next(item for item in result["checks"] if item["name"] == "hover_drift")
    assert not drift["passed"]


def test_leg_length_from_env():
    env = os.environ.copy()
    env["E2E_LEG_LENGTH_M"] = "1.0"
    thresholds = load_thresholds(env)
    assert thresholds.leg_length_m == 1.0
    result = grade_mission(_perfect(leg=1.0), thresholds)
    assert result["passed"], result["checks"]


def test_short_leg_fails_default_threshold():
    result = grade_mission(_perfect(leg=0.1))
    leg = next(item for item in result["checks"] if item["name"] == "leg1")
    assert not leg["passed"]
