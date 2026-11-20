"""A crash during the vision-loss landing fails the run. VisionLost does not."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from px4_offboard import FlightCrash, FlightWatch, VisionLost  # noqa: E402


def _watch() -> FlightWatch:
    watch = FlightWatch(commanding=False)
    watch.px4_n = 1.0
    watch.px4_e = 2.0
    watch.px4_d = -1.5
    return watch


def test_vision_lost_still_lands():
    watch = _watch()
    calls = []
    watch._wait_sim = lambda *_args, **_kwargs: (_ for _ in ()).throw(VisionLost("lost"))
    watch._land = lambda: calls.append("land")
    watch._disarm = lambda: calls.append("disarm")
    watch._brake_hold_land()
    assert calls == ["land", "disarm"]


def test_flight_crash_during_the_hold_is_not_swallowed():
    watch = _watch()
    watch._wait_sim = lambda *_args, **_kwargs: (_ for _ in ()).throw(FlightCrash("tilt"))
    watch._land = lambda: pytest.fail("landed after a crash")
    with pytest.raises(FlightCrash) as caught:
        watch._brake_hold_land()
    assert caught.value.reason == "tilt"


def test_flight_crash_during_landing_is_not_swallowed():
    watch = _watch()
    watch._wait_sim = lambda *_args, **_kwargs: None
    watch._land = lambda: (_ for _ in ()).throw(FlightCrash("setpoint"))
    with pytest.raises(FlightCrash) as caught:
        watch._brake_hold_land()
    assert caught.value.reason == "setpoint"
