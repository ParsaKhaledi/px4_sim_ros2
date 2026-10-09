import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from grading import grade_mission, load_thresholds  # noqa: E402


def _sample(phase, x, y, z, yaw, t):
    return {"phase": phase, "x": x, "y": y, "z": z, "yaw": yaw, "t": t}


def _hold(phase, x, y, z, yaw, t0, t1, step=0.5):
    rows = []
    t = t0
    while t < t1 - 1e-9:
        rows.append(_sample(phase, x, y, z, yaw, round(t, 3)))
        t += step
    rows.append(_sample(phase, x, y, z, yaw, t1))
    return rows


def _perfect(leg=0.3):
    # Heading 90° points the step along +x.
    yaw0 = 90.0
    yaw1 = 270.0
    z = 2.1
    rows = [_sample("start", 0.0, 0.0, 0.1, yaw0, 0.0)]
    rows.append(_sample("takeoff", 0.0, 0.0, 1.0, yaw0, 1.0))
    rows.extend(_hold("takeoff", 0.0, 0.0, z, yaw0, 3.0, 5.0))
    rows.extend(_hold("hover", 0.0, 0.0, z, yaw0, 5.0, 15.0, step=2.5))
    rows.append(_sample("leg1", 0.0, 0.0, z, yaw0, 15.0))
    rows.append(_sample("leg1", leg * 0.5, 0.0, z, yaw0, 15.4))
    rows.extend(_hold("leg1", leg, 0.0, z, yaw0, 16.0, 17.0))
    rows.append(_sample("yaw", leg, 0.0, z, yaw0, 17.2))
    rows.append(_sample("yaw", leg, 0.0, z, 180.0, 17.6))
    rows.extend(_hold("yaw", leg, 0.0, z, yaw1, 18.0, 19.0))
    rows.append(_sample("leg2", leg, 0.0, z, yaw1, 19.0))
    rows.append(_sample("leg2", leg * 0.5, 0.0, z, yaw1, 19.4))
    rows.extend(_hold("leg2", 0.0, 0.0, z, yaw1, 20.0, 21.0))
    rows.append(_sample("land", 0.02, 0.0, 0.1, yaw1, 25.0))
    return rows


def _check(result, name):
    return next(item for item in result["checks"] if item["name"] == name)


def test_flight_mechanics_defaults():
    thresholds = load_thresholds({})
    assert thresholds.leg_tolerance_m == 0.03
    assert thresholds.overshoot_m == 0.03
    assert thresholds.settle_tolerance_m == 0.02
    assert thresholds.settle_hold_s == 1.0
    assert thresholds.settle_timeout_s == 4.0
    assert thresholds.yaw_tolerance_deg == 5.0
    assert thresholds.yaw_settle_deg == 3.0
    assert thresholds.hover_height_band_m == 0.05
    assert thresholds.hover_height_hold_s == 2.0
    assert thresholds.return_tolerance_m == 0.05


def test_perfect_mission_passes():
    result = grade_mission(_perfect())
    assert result["passed"], result["checks"]


def test_hover_drift_fails():
    rows = _perfect()
    rows.append(_sample("hover", 0.2, 0.0, 2.1, 90.0, 15.2))
    result = grade_mission(rows)
    assert not _check(result, "hover_drift")["passed"]


def test_leg_length_from_env():
    env = os.environ.copy()
    env["E2E_LEG_LENGTH_M"] = "1.0"
    thresholds = load_thresholds(env)
    assert thresholds.leg_length_m == 1.0
    result = grade_mission(_perfect(leg=1.0), thresholds)
    assert result["passed"], result["checks"]


def test_origin_is_the_last_start_sample():
    rows = [_sample("start", 0.0, 0.0, 4.0, 90.0, -1.0)]
    rows.extend(_perfect())
    result = grade_mission(rows)
    assert _check(result, "height")["passed"], _check(result, "height")


def test_short_leg_fails_default_threshold():
    result = grade_mission(_perfect(leg=0.1))
    assert not _check(result, "leg1")["passed"]


def test_overshoot_fails_when_the_final_point_is_exact():
    rows = _perfect()
    index = next(i for i, row in enumerate(rows) if row["phase"] == "leg1" and row["t"] >= 16.0)
    rows.insert(index, _sample("leg1", 0.34, 0.0, 2.1, 90.0, 15.6))
    result = grade_mission(rows)
    overshoot = _check(result, "leg1_overshoot")
    assert not overshoot["passed"]
    assert overshoot["measured"] >= 0.03
    assert not result["passed"]


def test_settle_that_finishes_after_four_seconds_fails():
    rows = [row for row in _perfect() if row["phase"] != "leg1"]
    z = 2.1
    rows.extend(
        [
            _sample("leg1", 0.0, 0.0, z, 90.0, 15.0),
            _sample("leg1", 0.15, 0.0, z, 90.0, 17.0),
            _sample("leg1", 0.3, 0.0, z, 90.0, 18.2),
            _sample("leg1", 0.3, 0.0, z, 90.0, 18.7),
            _sample("leg1", 0.3, 0.0, z, 90.0, 19.2),
        ]
    )
    result = grade_mission(rows)
    settle = _check(result, "leg1_settle")
    assert not settle["passed"]
    assert _check(result, "leg1")["passed"]


def test_yaw_overshoot_is_five_degrees():
    rows = _perfect()
    rows.append(_sample("yaw", 0.3, 0.0, 2.1, 276.0, 17.8))
    result = grade_mission(rows)
    overshoot = _check(result, "yaw_overshoot")
    assert not overshoot["passed"]
    assert overshoot["measured"] > 5.0
    assert _check(result, "yaw_settle")["passed"]


def test_yaw_outside_three_degrees_does_not_settle():
    rows = [row for row in _perfect() if row["phase"] != "yaw"]
    z = 2.1
    rows.extend(_hold("yaw", 0.3, 0.0, z, 274.0, 17.2, 19.0))
    result = grade_mission(rows)
    assert not _check(result, "yaw_settle")["passed"]


def test_hover_clock_waits_for_the_height_hold():
    rows = [row for row in _perfect() if row["phase"] != "takeoff"]
    rows.extend(_hold("takeoff", 0.0, 0.0, 1.5, 90.0, 3.0, 5.0))
    result = grade_mission(rows)
    assert not _check(result, "hover_ready")["passed"]
