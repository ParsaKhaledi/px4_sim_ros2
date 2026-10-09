#!/usr/bin/env python3
"""Out-and-back flight.

Uses px4_control.Drone when that package imports. Otherwise flies with
the PX4 offboard topics in px4_offboard.py. A missing package is not a
skip. Crash rules and the sim restart live around this one attempt.
"""

from __future__ import annotations

import importlib
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests" / "e2e"))

from crash_monitor import load_crash_thresholds  # noqa: E402
from grading import load_thresholds  # noqa: E402
from px4_offboard import FlightCrash, FlightSetupError, FlightWatch, run_px4_mission  # noqa: E402


DRONE_CANDIDATES = (
    ("px4_control", "Drone"),
    ("px4_control.drone", "Drone"),
    ("px4_control.api", "Drone"),
)


def import_drone():
    for module_name, attr in DRONE_CANDIDATES:
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        drone_cls = getattr(module, attr, None)
        if drone_cls is not None:
            return drone_cls
    return None


def driver_name() -> str:
    if import_drone() is None:
        return "px4_offboard"
    return "px4_control"


def _call(obj, names, *args, **kwargs):
    for name in names:
        func = getattr(obj, name, None)
        if func is None:
            continue
        try:
            return func(*args, **kwargs)
        except TypeError:
            continue
    raise FlightSetupError(f"Drone API has no usable method among {names}")


def _result_path() -> Path:
    override = os.environ.get("E2E_RESULT_PATH")
    if override:
        return Path(override)
    container = Path("/home/px4/volume/logs/flights/trajectory.json")
    if container.parent.exists():
        return container
    return ROOT / "logs" / "flights" / "trajectory.json"


def _write(payload: dict) -> None:
    path = _result_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _dump(thresholds) -> dict:
    return {
        "takeoff_height_m": thresholds.takeoff_height_m,
        "hover_s": thresholds.hover_s,
        "leg_length_m": thresholds.leg_length_m,
        "hover_drift_m": thresholds.hover_drift_m,
        "leg_tolerance_m": thresholds.leg_tolerance_m,
        "yaw_tolerance_deg": thresholds.yaw_tolerance_deg,
        "yaw_settle_deg": thresholds.yaw_settle_deg,
        "return_tolerance_m": thresholds.return_tolerance_m,
        "height_tolerance_m": thresholds.height_tolerance_m,
        "overshoot_m": thresholds.overshoot_m,
        "settle_tolerance_m": thresholds.settle_tolerance_m,
        "settle_hold_s": thresholds.settle_hold_s,
        "settle_timeout_s": thresholds.settle_timeout_s,
        "hover_height_band_m": thresholds.hover_height_band_m,
        "hover_height_hold_s": thresholds.hover_height_hold_s,
        "max_retries": load_crash_thresholds().max_retries,
    }


def run_drone_mission(drone_cls, thresholds) -> dict:
    """Fly with the control package, and still watch for a crash."""

    watch = FlightWatch(thresholds=thresholds, commanding=False)
    watch.start()
    drone = drone_cls()
    try:
        watch.set_phase("start")
        time.sleep(1.0)
        if hasattr(drone, "preflight"):
            _call(drone, ("preflight",))
        watch.set_phase("arm")
        _call(drone, ("arm",))
        watch.raise_if_crashed()
        watch.set_phase("takeoff")
        try:
            _call(drone, ("takeoff",), thresholds.takeoff_height_m)
        except FlightSetupError:
            _call(drone, ("takeoff",), height_m=thresholds.takeoff_height_m)
        watch.set_phase("hover")
        if hasattr(drone, "hold"):
            try:
                _call(drone, ("hold",), thresholds.hover_s)
            except FlightSetupError:
                time.sleep(thresholds.hover_s)
        else:
            time.sleep(thresholds.hover_s)
        watch.raise_if_crashed()
        watch.set_phase("leg1")
        try:
            _call(drone, ("move_forward",), thresholds.leg_length_m)
        except FlightSetupError:
            _call(drone, ("move_forward",), distance_m=thresholds.leg_length_m)
        watch.set_phase("yaw")
        try:
            _call(drone, ("turn",), 180.0)
        except FlightSetupError:
            _call(drone, ("turn",), degrees=180.0)
        watch.raise_if_crashed()
        watch.set_phase("leg2")
        try:
            _call(drone, ("move_forward",), thresholds.leg_length_m)
        except FlightSetupError:
            _call(drone, ("move_forward",), distance_m=thresholds.leg_length_m)
        watch.set_phase("land")
        _call(drone, ("land",))
        if hasattr(drone, "disarm"):
            watch.set_phase("disarm")
            _call(drone, ("disarm",))
        time.sleep(1.0)
        watch.raise_if_crashed()
    finally:
        watch.stop()
    graded = watch._result(crashed=False)
    graded["driver"] = "px4_control"
    graded["thresholds"] = _dump(thresholds)
    return graded


def run_mission() -> dict:
    thresholds = load_thresholds()
    drone_cls = import_drone()
    if drone_cls is None:
        result = run_px4_mission(thresholds)
        result["thresholds"] = _dump(thresholds)
        return result
    return run_drone_mission(drone_cls, thresholds)


def main() -> int:
    try:
        result = run_mission()
    except FlightCrash as exc:
        payload = exc.payload(load_thresholds(), driver_name())
        _write(payload)
        print(json.dumps({"status": "crashed", "crash_reason": exc.reason, "snapshot": exc.snapshot}, indent=2))
        return 2
    except FlightSetupError as exc:
        payload = {"status": "failed", "passed": False, "driver": driver_name(), "reason": str(exc)}
        _write(payload)
        print(f"SETUP: {exc}", file=sys.stderr)
        return 3
    except Exception as exc:
        payload = {"status": "failed", "passed": False, "driver": driver_name(), "reason": str(exc)}
        _write(payload)
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    _write(result)
    summary = {
        "status": result.get("status"),
        "passed": result.get("passed"),
        "driver": result.get("driver"),
        "checks": result.get("checks"),
        "px4_position_error_m": result.get("px4_position_error_m"),
        "gz_rtf": result.get("gz_rtf"),
        "cameras": result.get("cameras"),
    }
    print(json.dumps(summary, indent=2))
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    sys.exit(main())
