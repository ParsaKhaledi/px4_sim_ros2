"""Dropout grades brake, hold, and land. The yaw check uses the commanded direction."""

from grading import grade_dropout, yaw_passed_command_direction


def _row(phase, x, y, z, yaw, t):
    return {"phase": phase, "x": x, "y": y, "z": z, "yaw": yaw, "t": t}


def test_dropout_passes_only_brake_hold_land():
    rows = [
        _row("start", 0.0, 0.0, 0.1, 37.0, 0.0),
        _row("hover", 0.0, 0.0, 2.0, 37.0, 12.0),
        _row("vision_brake", 1.0, 0.2, 2.0, 10.0, 20.0),
        _row("vision_brake", 1.05, 0.2, 2.0, 10.0, 22.0),
        _row("land", 1.05, 0.2, 0.15, 10.0, 28.0),
    ]
    result = grade_dropout(rows)
    assert result["passed"], result["checks"]
    assert {item["name"] for item in result["checks"]} == {"brake", "hold", "land"}


def test_dropout_fails_without_a_brake():
    rows = [
        _row("start", 0.0, 0.0, 0.1, 0.0, 0.0),
        _row("land", 0.0, 0.0, 0.1, 0.0, 10.0),
    ]
    assert not grade_dropout(rows)["passed"]


def test_yaw_must_pass_ninety_in_the_commanded_direction():
    hover = [_row("hover", 0.0, 0.0, 2.0, 37.0, 10.0)]
    commanded = hover + [
        _row("yaw", 0.0, 0.0, 2.0, 37.0, 17.0),
        _row("yaw", 0.0, 0.0, 2.0, 0.0, 17.4),
        _row("yaw", 0.0, 0.0, 2.0, -53.0, 17.8),
        _row("yaw", 0.0, 0.0, 2.0, -143.0, 18.2),
    ]
    assert yaw_passed_command_direction(commanded)["passed"]
    reversed_turn = hover + [
        _row("yaw", 0.0, 0.0, 2.0, 37.0, 17.0),
        _row("yaw", 0.0, 0.0, 2.0, 90.0, 17.4),
        _row("yaw", 0.0, 0.0, 2.0, 127.0, 17.8),
        _row("yaw", 0.0, 0.0, 2.0, 217.0, 18.2),
    ]
    assert not yaw_passed_command_direction(reversed_turn)["passed"]
