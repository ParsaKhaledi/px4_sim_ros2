"""Turn measurements into pass/fail checks against the shared E2E limits."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from e2e_limits import E2ELimits
from flight_analysis.metrics import VISION_LEAK_M
from flight_analysis.metrics import ErrorStats
from flight_analysis.metrics import FlightMetrics
from flight_analysis.metrics import LegMetrics


def grade(metrics: FlightMetrics, limits: E2ELimits) -> dict[str, Any]:
    """Build the check list. A skipped check is not a failure."""

    checks: list[dict[str, Any]] = []
    for row in metrics.legs:
        checks.extend(_grade_leg(row, limits))
    checks.extend(_grade_flight(metrics, limits))
    return {
        "passed": all(bool(check["passed"]) for check in checks),
        "checks": checks,
        "limits": asdict(limits),
    }


def _grade_leg(row: LegMetrics, limits: E2ELimits) -> list[dict[str, Any]]:
    """Pass/fail for one leg's overshoot, settling, and stopping distance."""

    leg = row.leg
    checks: list[dict[str, Any]] = []
    if row.overshoot_m is not None:
        checks.append(
            _check(
                "overshoot",
                row.overshoot_m <= limits.overshoot_m,
                row.overshoot_m,
                limits.overshoot_m,
                _cm_reason(
                    "peak",
                    row.overshoot_m,
                    limits.overshoot_m,
                    "past the position target",
                ),
                leg=leg.index,
                kind=leg.kind,
            )
        )
    if row.yaw_overshoot_deg is not None:
        checks.append(
            _check(
                "yaw_overshoot",
                row.yaw_overshoot_deg <= limits.yaw_tolerance_deg,
                row.yaw_overshoot_deg,
                limits.yaw_tolerance_deg,
                (
                    f"peak {row.yaw_overshoot_deg:.2f} deg past the yaw target, "
                    f"limit {limits.yaw_tolerance_deg:.2f} deg"
                ),
                leg=leg.index,
                kind=leg.kind,
            )
        )
    if leg.kind in {"takeoff", "land", "translate", "hover"}:
        checks.append(_settle_check(row, limits, yaw=False))
    if leg.kind == "yaw":
        checks.append(_settle_check(row, limits, yaw=True))
    if row.stopping_distance_m is not None:
        checks.append(
            _check(
                "stopping_distance",
                row.stopping_distance_m <= limits.overshoot_m,
                row.stopping_distance_m,
                limits.overshoot_m,
                _cm_reason(
                    "still",
                    row.stopping_distance_m,
                    limits.overshoot_m,
                    "past the target once stopped",
                ),
                leg=leg.index,
                kind=leg.kind,
            )
        )
    return checks


def _settle_check(row: LegMetrics, limits: E2ELimits, yaw: bool) -> dict[str, Any]:
    """Settling check. ``None`` means the hold never finished before the timeout."""

    if yaw:
        measured = row.yaw_settling_time_s
        band = limits.yaw_settle_deg
        unit = "deg"
        name = "yaw_settle"
    else:
        measured = row.settling_time_s
        band = limits.settle_tolerance_m
        unit = "m"
        name = "settle"
    passed = measured is not None and measured <= limits.settle_timeout_s
    if measured is None:
        reason = (
            f"did not stay within ±{band:.3f} {unit} for {limits.settle_hold_s:.2f} s "
            f"before {limits.settle_timeout_s:.2f} s"
        )
    else:
        reason = (
            f"inside ±{band:.3f} {unit} for {limits.settle_hold_s:.2f} s "
            f"by {measured:.2f} s (timeout {limits.settle_timeout_s:.2f} s)"
        )
    return _check(
        name,
        passed,
        measured,
        limits.settle_timeout_s,
        reason,
        leg=row.leg.index,
        kind=row.leg.kind,
    )


def _grade_flight(metrics: FlightMetrics, limits: E2ELimits) -> list[dict[str, Any]]:
    """Flight-level checks: return, tilt, saturation, height, vision, rates."""

    checks = [
        _rollup(metrics, limits),
        _return_check(metrics, limits),
        _tilt_check(metrics),
        _saturation_check(metrics),
        _height_check(metrics, limits),
        _leak_check(metrics),
        _rates_check(metrics),
        _vision_check(metrics),
        _flags_check(metrics),
    ]
    return [check for check in checks if check is not None]


def _rollup(metrics: FlightMetrics, limits: E2ELimits) -> dict[str, Any] | None:
    """Worst position overshoot on the flight, so a summary has one number."""

    rows = [row for row in metrics.legs if row.overshoot_m is not None]
    if not rows:
        return None
    worst = max(rows, key=lambda row: row.overshoot_m or 0.0)
    measured = worst.overshoot_m or 0.0
    return _check(
        "overshoot",
        measured <= limits.overshoot_m,
        measured,
        limits.overshoot_m,
        f"worst leg {worst.leg.index} ({worst.leg.kind}): " + _cm_reason(
            "peak", measured, limits.overshoot_m, "past the position target"
        ),
        scope="flight",
    )


def _return_check(metrics: FlightMetrics, limits: E2ELimits) -> dict[str, Any]:
    """Horizontal distance from the outbound start back to the end of the return."""

    if metrics.return_distance_m is None:
        return _check(
            "return_to_start",
            True,
            None,
            limits.return_tolerance_m,
            "no return leg to grade",
            skipped=True,
            scope="flight",
        )
    distance = metrics.return_distance_m
    return _check(
        "return_to_start",
        distance <= limits.return_tolerance_m,
        distance,
        limits.return_tolerance_m,
        _cm_reason("finished", distance, limits.return_tolerance_m, "from the outbound start"),
        scope="flight",
    )


def _tilt_check(metrics: FlightMetrics) -> dict[str, Any]:
    """Peak tilt against the logged ``MPC_TILTMAX_AIR`` parameter, in degrees."""

    if metrics.tilt_peak_deg is None:
        return _check(
            "tilt",
            True,
            None,
            metrics.tilt_limit_deg,
            "vehicle_attitude was not logged",
            skipped=True,
            scope="flight",
        )
    if metrics.tilt_limit_deg is None:
        return _check(
            "tilt",
            True,
            metrics.tilt_peak_deg,
            None,
            f"peak tilt {metrics.tilt_peak_deg:.2f} deg; MPC_TILTMAX_AIR was not logged",
            skipped=True,
            scope="flight",
        )
    passed = metrics.tilt_peak_deg <= metrics.tilt_limit_deg
    return _check(
        "tilt",
        passed,
        metrics.tilt_peak_deg,
        metrics.tilt_limit_deg,
        (
            f"peak tilt {metrics.tilt_peak_deg:.2f} deg, "
            f"MPC_TILTMAX_AIR {metrics.tilt_limit_deg:.2f} deg"
        ),
        scope="flight",
    )


def _saturation_check(metrics: FlightMetrics) -> dict[str, Any]:
    """Fail when a logged motor sits on its thrust stop."""

    if metrics.saturation is None:
        return _check(
            "actuator_saturation",
            True,
            None,
            None,
            "actuator_motors and actuator_outputs were not logged",
            skipped=True,
            scope="flight",
        )
    fraction = float(metrics.saturation["fraction"])
    return _check(
        "actuator_saturation",
        fraction == 0.0,
        fraction,
        0.0,
        f"{metrics.saturation['kind']} outputs on the stop for {fraction:.1%} of samples",
        scope="flight",
    )


def _height_check(metrics: FlightMetrics, limits: E2ELimits) -> dict[str, Any]:
    """First height disagreement between the estimate and ground truth."""

    if not metrics.ground_truth_logged:
        return _check(
            "height_divergence",
            True,
            None,
            limits.height_tolerance_m,
            "no ground truth; height divergence was not graded",
            skipped=True,
            scope="flight",
        )
    event = metrics.height_divergence
    if event is None:
        return _check(
            "height_divergence",
            True,
            0.0,
            limits.height_tolerance_m,
            f"estimate and ground truth height stayed within {limits.height_tolerance_m:.2f} m",
            scope="flight",
        )
    return _check(
        "height_divergence",
        False,
        event["error_m"],
        limits.height_tolerance_m,
        (
            f"at t={event['time_s']:.2f} s estimate height {event['estimate_up_m']:.2f} m "
            f"and ground truth {event['truth_up_m']:.2f} m "
            f"differ by {event['error_m']:.2f} m "
            f"(limit {limits.height_tolerance_m:.2f} m)"
        ),
        scope="flight",
    )


def _leak_check(metrics: FlightMetrics) -> dict[str, Any]:
    """Suspect when a vision hover matches ground truth to about 1 mm."""

    leak = metrics.leak
    checked = bool(leak.get("checked"))
    suspect = bool(leak.get("suspect"))
    measured = leak.get("max_error_m")
    reason = str(leak.get("reason") or "")
    if not checked:
        return _check(
            "vision_ground_truth_leak",
            True,
            measured if isinstance(measured, float) else None,
            VISION_LEAK_M,
            reason,
            skipped=True,
            scope="flight",
        )
    if suspect:
        return _check(
            "vision_ground_truth_leak",
            False,
            float(measured) if isinstance(measured, float) else 0.0,
            VISION_LEAK_M,
            "vision-mode hover matched ground truth within 1 mm; "
            "ground truth may have been fused as vision",
            scope="flight",
        )
    detail = reason
    if isinstance(measured, float):
        detail = f"{reason}; peak error {measured * 100:.2f} cm"
    return _check(
        "vision_ground_truth_leak",
        True,
        measured if isinstance(measured, float) else None,
        VISION_LEAK_M,
        detail,
        scope="flight",
    )


def _rates_check(metrics: FlightMetrics) -> dict[str, Any]:
    """Body-rate RMS and dominant frequency. There is no E2E limit for this."""

    if metrics.rates is None:
        return _check(
            "body_rates",
            True,
            None,
            None,
            "vehicle_angular_velocity was not logged",
            skipped=True,
            scope="flight",
        )
    rms = metrics.rates["rms_rad_s"]
    hz = metrics.rates["dominant_hz"]
    hz_text = "n/a" if hz is None else f"{hz:.2f} Hz"
    return _check(
        "body_rates",
        True,
        rms if isinstance(rms, float) else None,
        None,
        f"rate RMS {rms:.3f} rad/s, dominant frequency {hz_text}; no E2E oscillation limit",
        scope="flight",
    )


def _vision_check(metrics: FlightMetrics) -> dict[str, Any]:
    """Vision delay summary. There is no E2E limit for the delay."""

    if metrics.vision_delay is None:
        return _check(
            "vision_delay",
            True,
            None,
            None,
            "vehicle_visual_odometry was not logged",
            skipped=True,
            scope="flight",
        )
    delay = metrics.vision_delay
    return _check(
        "vision_delay",
        True,
        delay["max_s"],
        None,
        (
            f"vision delay median {delay['median_s'] * 1e3:.1f} ms, "
            f"p95 {delay['p95_s'] * 1e3:.1f} ms, max {delay['max_s'] * 1e3:.1f} ms"
        ),
        scope="flight",
    )


def _flags_check(metrics: FlightMetrics) -> dict[str, Any]:
    """Note when estimator source flags were not in the log."""

    if metrics.estimator_flags_logged:
        return _check(
            "estimator_flags",
            True,
            None,
            None,
            "estimator_status_flags logged; see each leg",
            scope="flight",
        )
    return _check(
        "estimator_flags",
        True,
        None,
        None,
        "estimator_status_flags was not logged",
        skipped=True,
        scope="flight",
    )


def _cm_reason(verb: str, measured_m: float, limit_m: float, tail: str) -> str:
    """Phrase a metre measurement in centimetres for the check reason."""

    return f"{verb} {measured_m * 100:.2f} cm {tail} (limit {limit_m * 100:.2f} cm)"


def _check(
    name: str,
    passed: bool,
    measured: float | None,
    limit: float | None,
    reason: str,
    leg: int | None = None,
    kind: str | None = None,
    skipped: bool = False,
    scope: str = "leg",
) -> dict[str, Any]:
    """One machine-readable check. Skipped checks pass so they do not fail the run."""

    return {
        "name": name,
        "scope": scope if leg is None else "leg",
        "leg": leg,
        "kind": kind,
        "passed": True if skipped else bool(passed),
        "skipped": skipped,
        "measured": measured,
        "limit": limit,
        "reason": reason,
    }


def error_dict(stats: ErrorStats | None) -> dict[str, float | None] | None:
    """JSON form of an error summary."""

    if stats is None:
        return None
    return asdict(stats)
