import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from grading import (  # noqa: E402
    _grade_leg,
    _leg_axis,
    flight_height_reference,
    grade_mission,
    load_thresholds,
)


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
    # ENU yaw 0 faces east, so the out leg is +x. After 180 deg the nose faces west.
    yaw0 = 0.0
    yaw1 = 180.0
    z = 2.1
    rows = [_sample("start", 0.0, 0.0, 0.1, yaw0, 0.0)]
    rows.append(_sample("takeoff", 0.0, 0.0, 1.0, yaw0, 1.0))
    rows.extend(_hold("takeoff", 0.0, 0.0, z, yaw0, 3.0, 5.0))
    rows.extend(_hold("hover", 0.0, 0.0, z, yaw0, 5.0, 15.0, step=2.5))
    rows.append(_sample("leg1", 0.0, 0.0, z, yaw0, 15.0))
    rows.append(_sample("leg1", leg * 0.5, 0.0, z, yaw0, 15.4))
    rows.extend(_hold("leg1", leg, 0.0, z, yaw0, 16.0, 17.0))
    rows.append(_sample("yaw", leg, 0.0, z, yaw0, 17.2))
    rows.append(_sample("yaw", leg, 0.0, z, 90.0, 17.6))
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
    rows.append(_sample("yaw", 0.3, 0.0, 2.1, 186.0, 17.8))
    result = grade_mission(rows)
    overshoot = _check(result, "yaw_overshoot")
    assert not overshoot["passed"]
    assert overshoot["measured"] > 5.0
    assert _check(result, "yaw_settle")["passed"]


def test_yaw_outside_three_degrees_does_not_settle():
    rows = [row for row in _perfect() if row["phase"] != "yaw"]
    z = 2.1
    rows.extend(_hold("yaw", 0.3, 0.0, z, 184.0, 17.2, 19.0))
    result = grade_mission(rows)
    assert not _check(result, "yaw_settle")["passed"]


def test_hover_clock_waits_for_the_height_hold():
    rows = [row for row in _perfect() if row["phase"] != "takeoff"]
    rows.extend(_hold("takeoff", 0.0, 0.0, 1.5, 90.0, 3.0, 5.0))
    result = grade_mission(rows)
    assert not _check(result, "hover_ready")["passed"]


def test_final_yaw_is_the_change_from_takeoff():
    result = grade_mission(_perfect())
    final = _check(result, "final_yaw")
    assert final["passed"], final
    assert final["measured"] == 0.0
    assert result["yaw"]["takeoff_yaw_deg"] == 0.0
    assert result["yaw"]["final_yaw_deg"] == 180.0
    assert result["yaw"]["yaw_change_deg"] == 180.0
    # Same absolute heading as a finished turn, but takeoff was already there.
    rows = _perfect()
    for row in rows:
        if row["phase"] != "yaw":
            row["yaw"] = 180.0
    missed = grade_mission(rows)
    final = _check(missed, "final_yaw")
    assert not final["passed"]
    assert final["measured"] == 180.0


def test_height_zero_is_prearm_base_link_not_the_ekf_origin():
    rows = _perfect()
    # Model origin 0.1 m, base_link 0.34 m. The 0.24 m offset is on both
    # the pre-arm pose and the hover, so the climb stays 2.0 m.
    for row in rows:
        if row["phase"] == "start":
            row["px4_d"] = -0.24
        elif row["phase"] == "hover":
            row["px4_d"] = -2.24
    result = grade_mission(rows)
    assert _check(result, "height")["measured"] == 0.0
    reference = flight_height_reference(rows)
    assert reference["ground_truth"]["prearm_base_link_z_m"] == 0.34
    assert reference["ground_truth"]["hover_above_prearm_m"] == 2.0
    assert reference["ekf"]["hover_above_prearm_m"] == 2.0
    assert reference["ekf"]["prearm_down_m"] == -0.24


def test_leg_axis_follows_enu_at_37_deg():
    yaw = 37.0
    heading = math.radians(yaw)
    axis = (math.cos(heading), math.sin(heading))
    assert math.isclose(_leg_axis(0.0)[0], 1.0, abs_tol=1e-9)
    assert math.isclose(_leg_axis(0.0)[1], 0.0, abs_tol=1e-9)
    assert math.isclose(_leg_axis(90.0)[0], 0.0, abs_tol=1e-9)
    assert math.isclose(_leg_axis(90.0)[1], 1.0, abs_tol=1e-9)
    assert math.isclose(_leg_axis(yaw)[0], axis[0], abs_tol=1e-9)
    assert math.isclose(_leg_axis(yaw)[1], axis[1], abs_tol=1e-9)
    length = 0.3
    start = {"x": -1.7, "y": 2.4, "z": 2.0, "yaw": yaw}
    target = (start["x"] + length * axis[0], start["y"] + length * axis[1])
    past = (target[0] + 0.04 * axis[0], target[1] + 0.04 * axis[1])
    rows = [
        {"phase": "leg1", "x": start["x"], "y": start["y"], "z": 2.0, "yaw": yaw, "t": 1.0},
        {"phase": "leg1", "x": past[0], "y": past[1], "z": 2.0, "yaw": yaw, "t": 1.4},
        {"phase": "leg1", "x": target[0], "y": target[1], "z": 2.0, "yaw": yaw, "t": 2.0},
        {"phase": "leg1", "x": target[0], "y": target[1], "z": 2.0, "yaw": yaw, "t": 3.2},
    ]
    checks = []
    _grade_leg(checks, "leg1", rows, start, load_thresholds({}))
    overshoot = next(item for item in checks if item["name"] == "leg1_overshoot")
    assert math.isclose(overshoot["measured"], 0.04, abs_tol=1e-4)
    assert not overshoot["passed"]
    leg = next(item for item in checks if item["name"] == "leg1")
    assert leg["passed"]
