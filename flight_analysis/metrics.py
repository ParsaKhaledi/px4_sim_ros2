"""Control numbers for one flight. Limits are applied later, in grading."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from flight_analysis.frames import wrap_pi
from flight_analysis.log import Series
from flight_analysis.log import Track
from flight_analysis.segment import AIRBORNE_UP_M
from flight_analysis.segment import Leg


# A multicopter this slow is stopped, not still sliding into the target.
STOP_SPEED_M_S = 0.05
STOP_HOLD_S = 0.2
# Motor output in actuator_motors: 0 is idle and 1 is full thrust.
MOTOR_SATURATION = 0.95
# PX4's usual PWM rails are 1000 and 2000 µs. Within 100 µs is on the stop.
PWM_LOW_US = 1100.0
PWM_HIGH_US = 1900.0
# A real vision fix wanders by centimetres. A whole hover inside a millimetre
# of ground truth means the truth was probably fused as if it were vision.
VISION_LEAK_M = 0.001
HOVER_SPEED_M_S = 0.15


@dataclass
class ErrorStats:
    """Setpoint-versus-estimate, or estimate-versus-truth, on one window."""

    horizontal_rms_m: float
    horizontal_peak_m: float
    vertical_rms_m: float
    vertical_peak_m: float
    yaw_rms_deg: float | None
    yaw_peak_deg: float | None


@dataclass
class LegMetrics:
    """Geometry of one leg, before pass/fail."""

    leg: Leg
    overshoot_m: float | None
    yaw_overshoot_deg: float | None
    settling_time_s: float | None
    yaw_settling_time_s: float | None
    stopping_distance_m: float | None
    tracking: ErrorStats | None
    estimation: ErrorStats | None
    flags: dict[str, dict[str, float | bool]] | None


@dataclass
class FlightMetrics:
    """Whole-flight measurements. ``None`` means that topic was not logged."""

    legs: list[LegMetrics]
    tracking: ErrorStats | None
    estimation: ErrorStats | None
    tilt_peak_deg: float | None
    tilt_limit_deg: float | None
    saturation: dict[str, float | str] | None
    rates: dict[str, float | None] | None
    height_divergence: dict[str, float] | None
    vision_delay: dict[str, float] | None
    estimator_flags_logged: bool
    leak: dict[str, object]
    ground_truth_logged: bool
    return_distance_m: float | None


def measure_flight(
    command: Track,
    estimate: Track,
    legs: list[Leg],
    ground_truth: Track | None,
    tilt_time_s: np.ndarray | None,
    tilt_rad: np.ndarray | None,
    tilt_limit_deg: float | None,
    rate_time_s: np.ndarray | None,
    rates_rad_s: np.ndarray | None,
    actuator: tuple[str, np.ndarray, np.ndarray] | None,
    flags: Series | None,
    vision_delay_s: np.ndarray | None,
    settle_hold_s: float,
    settle_timeout_s: float,
    settle_tolerance_m: float,
    yaw_settle_deg: float,
    height_tolerance_m: float,
) -> FlightMetrics:
    """Measure every leg and the whole flight. This does not decide pass/fail."""

    leg_rows = [
        _measure_leg(
            leg,
            command,
            estimate,
            ground_truth,
            flags,
            settle_hold_s,
            settle_timeout_s,
            settle_tolerance_m,
            yaw_settle_deg,
        )
        for leg in legs
    ]
    return FlightMetrics(
        legs=leg_rows,
        tracking=_window_error(estimate, command, estimate.time_s[0], estimate.time_s[-1]),
        estimation=_estimation(estimate, ground_truth, estimate.time_s[0], estimate.time_s[-1]),
        tilt_peak_deg=_tilt_peak(tilt_rad),
        tilt_limit_deg=tilt_limit_deg,
        saturation=_saturation(actuator),
        rates=_rates(rate_time_s, rates_rad_s),
        height_divergence=_height_divergence(estimate, ground_truth, height_tolerance_m),
        vision_delay=_vision_delay(vision_delay_s),
        estimator_flags_logged=flags is not None,
        leak=_leak(legs, estimate, ground_truth, flags),
        ground_truth_logged=ground_truth is not None,
        return_distance_m=_return_distance(estimate, legs),
    )


def settling_time_s(
    time_s: np.ndarray,
    error: np.ndarray,
    t0: float,
    tolerance: float,
    hold_s: float,
    timeout_s: float,
) -> float | None:
    """Seconds from ``t0`` until ``hold_s`` inside the band, or ``None``.

    The hold has to finish by ``t0 + timeout_s``, so the returned time
    includes that hold. Samples outside ``[t0, t0 + timeout]`` are ignored.
    """

    deadline = t0 + timeout_s
    for index, start in enumerate(time_s):
        if start < t0 - 1e-9 or error[index] > tolerance:
            continue
        end = float(start) + hold_s
        if end > deadline + 1e-9:
            break
        window = (time_s >= start - 1e-9) & (time_s <= end + 1e-9)
        if not np.any(window):
            continue
        if float(time_s[window][-1]) < end - 1e-6:
            continue
        if np.all(error[window] <= tolerance):
            return float(end - t0)
    return None


def _measure_leg(
    leg: Leg,
    command: Track,
    estimate: Track,
    ground_truth: Track | None,
    flags: Series | None,
    settle_hold_s: float,
    settle_timeout_s: float,
    settle_tolerance_m: float,
    yaw_settle_deg: float,
) -> LegMetrics:
    """Overshoot, settling, stopping distance, and errors for one leg."""

    window = _slice(estimate, leg.t_start_s, leg.t_end_s)
    position = np.column_stack((window.north_m, window.east_m, window.down_m))
    start = np.array([leg.start_north_m, leg.start_east_m, leg.start_down_m])
    target = np.array([leg.target_north_m, leg.target_east_m, leg.target_down_m])
    grades_position = leg.kind in {"takeoff", "land", "translate", "hover"}
    grades_yaw = leg.kind == "yaw"
    overshoot = _overshoot_m(position, start, target) if grades_position else None
    yaw_overshoot = None
    if grades_yaw and window.time_s.shape[0]:
        yaw_overshoot = math.degrees(
            _yaw_overshoot_rad(window.yaw_rad, leg.start_yaw_rad, leg.target_yaw_rad)
        )
    settling = None
    if grades_position and window.time_s.shape[0]:
        distance = np.linalg.norm(position - target, axis=1)
        settling = settling_time_s(
            window.time_s,
            distance,
            leg.t_arrived_s,
            settle_tolerance_m,
            settle_hold_s,
            settle_timeout_s,
        )
    yaw_settling = None
    if grades_yaw and window.time_s.shape[0]:
        yaw_error = np.degrees(np.abs(_wrap_array(window.yaw_rad - leg.target_yaw_rad)))
        yaw_settling = settling_time_s(
            window.time_s,
            yaw_error,
            leg.t_arrived_s,
            yaw_settle_deg,
            settle_hold_s,
            settle_timeout_s,
        )
    stopping = None
    if leg.kind in {"takeoff", "land", "translate"} and window.time_s.shape[0]:
        stopping = _stopping_distance_m(window, start, target, leg.t_arrived_s)
    return LegMetrics(
        leg=leg,
        overshoot_m=overshoot,
        yaw_overshoot_deg=yaw_overshoot,
        settling_time_s=settling,
        yaw_settling_time_s=yaw_settling,
        stopping_distance_m=stopping,
        tracking=_window_error(estimate, command, leg.t_start_s, leg.t_end_s),
        estimation=_estimation(estimate, ground_truth, leg.t_start_s, leg.t_end_s),
        flags=_flags(flags, leg.t_start_s, leg.t_end_s),
    )


def _overshoot_m(position: np.ndarray, start: np.ndarray, target: np.ndarray) -> float:
    """How far past the target the estimate travelled, in metres.

    Overshoot is the peak along-track distance beyond the commanded move.
    A hold has no move, so this is the peak distance away from the point.
    """

    if position.shape[0] == 0:
        return 0.0
    delta = target - start
    length = float(np.linalg.norm(delta))
    if length < 1e-3:
        return float(np.max(np.linalg.norm(position - target, axis=1)))
    along = (position - start) @ (delta / length)
    return float(max(0.0, np.max(along) - length))


def _yaw_overshoot_rad(yaw: np.ndarray, start: float, target: float) -> float:
    """Peak yaw past the target, in the direction of the command, in radians.

    An exact half-turn is kept as +π so the sign does not flip at the branch cut.
    """

    direction = _signed_delta(start, target)
    errors = _wrap_array(yaw - target)
    if abs(direction) < math.radians(2.0):
        return float(np.max(np.abs(errors))) if errors.size else 0.0
    if direction > 0.0:
        return float(max(0.0, np.max(errors)))
    return float(max(0.0, -np.min(errors)))


def _stopping_distance_m(window: Track, start: np.ndarray, target: np.ndarray, t_arrived: float) -> float:
    """Distance still past the target once the vehicle has stopped.

    Overshoot can be larger: the vehicle may go past the target and come
    back before the speed stays under the stop threshold.
    """

    delta = target - start
    length = float(np.linalg.norm(delta))
    if length < 1e-3 or window.time_s.shape[0] == 0:
        return 0.0
    speed = _speed(window)
    after = window.time_s >= t_arrived - 1e-9
    if not np.any(after):
        return 0.0
    t_stop = _first_stop(window.time_s[after], speed[after])
    if t_stop is None:
        t_stop = float(window.time_s[after][-1])
    index = int(np.argmin(np.abs(window.time_s - t_stop)))
    position = np.array([window.north_m[index], window.east_m[index], window.down_m[index]])
    along = float((position - start) @ (delta / length))
    return float(max(0.0, along - length))


def _first_stop(time_s: np.ndarray, speed: np.ndarray) -> float | None:
    """Time when speed has stayed below the stop threshold for the hold."""

    low = speed <= STOP_SPEED_M_S
    for index, start in enumerate(time_s):
        if not low[index]:
            continue
        end = float(start) + STOP_HOLD_S
        window = (time_s >= start - 1e-9) & (time_s <= end + 1e-9)
        if float(time_s[window][-1]) < end - 1e-6:
            continue
        if np.all(low[window]):
            return end
    return None


def _speed(track: Track) -> np.ndarray:
    """Speed magnitude. Logged velocity is used when the topic has it."""

    if track.vn_m_s is not None and track.ve_m_s is not None and track.vd_m_s is not None:
        return np.sqrt(track.vn_m_s**2 + track.ve_m_s**2 + track.vd_m_s**2)
    dn = np.gradient(track.north_m, track.time_s)
    de = np.gradient(track.east_m, track.time_s)
    dd = np.gradient(track.down_m, track.time_s)
    return np.sqrt(dn**2 + de**2 + dd**2)


def _window_error(estimate: Track, reference: Track, t0: float, t1: float) -> ErrorStats | None:
    """RMS and peak error of ``estimate`` against ``reference`` on ``[t0, t1]``."""

    window = _slice(estimate, t0, t1)
    if window.time_s.shape[0] < 2 or reference.time_s.shape[0] < 2:
        return None
    overlap = (window.time_s >= reference.time_s[0]) & (window.time_s <= reference.time_s[-1])
    if int(np.count_nonzero(overlap)) < 2:
        return None
    time_s = window.time_s[overlap]
    north = np.interp(time_s, reference.time_s, reference.north_m)
    east = np.interp(time_s, reference.time_s, reference.east_m)
    down = np.interp(time_s, reference.time_s, reference.down_m)
    horizontal = np.hypot(window.north_m[overlap] - north, window.east_m[overlap] - east)
    vertical = np.abs(window.down_m[overlap] - down)
    yaw_rms = None
    yaw_peak = None
    if reference.yaw_rad.shape[0] == reference.time_s.shape[0]:
        yaw_ref = np.interp(time_s, reference.time_s, np.unwrap(reference.yaw_rad))
        yaw_err = np.degrees(np.abs(_wrap_array(window.yaw_rad[overlap] - yaw_ref)))
        yaw_rms = _rms(yaw_err)
        yaw_peak = float(np.max(yaw_err))
    return ErrorStats(
        horizontal_rms_m=_rms(horizontal),
        horizontal_peak_m=float(np.max(horizontal)),
        vertical_rms_m=_rms(vertical),
        vertical_peak_m=float(np.max(vertical)),
        yaw_rms_deg=yaw_rms,
        yaw_peak_deg=yaw_peak,
    )


def _estimation(
    estimate: Track,
    ground_truth: Track | None,
    t0: float,
    t1: float,
) -> ErrorStats | None:
    """Estimate versus ground truth. ``None`` when truth was not logged."""

    if ground_truth is None:
        return None
    return _window_error(estimate, ground_truth, t0, t1)


def _height_divergence(
    estimate: Track,
    ground_truth: Track | None,
    tolerance_m: float,
) -> dict[str, float] | None:
    """First time estimate and truth height disagree by more than ``tolerance_m``.

    Height is up-positive. In the failure this is meant to catch, the
    estimate sits on the setpoint while the vehicle keeps climbing.
    """

    if ground_truth is None or ground_truth.time_s.shape[0] < 2:
        return None
    overlap = (estimate.time_s >= ground_truth.time_s[0]) & (
        estimate.time_s <= ground_truth.time_s[-1]
    )
    if not np.any(overlap):
        return None
    time_s = estimate.time_s[overlap]
    truth_down = np.interp(time_s, ground_truth.time_s, ground_truth.down_m)
    estimate_up = -estimate.down_m[overlap]
    truth_up = -truth_down
    error = np.abs(estimate_up - truth_up)
    hits = np.flatnonzero(error > tolerance_m)
    if hits.size == 0:
        return None
    index = int(hits[0])
    return {
        "time_s": float(time_s[index]),
        "estimate_up_m": float(estimate_up[index]),
        "truth_up_m": float(truth_up[index]),
        "error_m": float(error[index]),
    }


def _tilt_peak(tilt_rad: np.ndarray | None) -> float | None:
    """Peak tilt in degrees, or ``None`` when attitude was not logged."""

    if tilt_rad is None or tilt_rad.size == 0:
        return None
    return float(np.degrees(np.max(tilt_rad)))


def _saturation(actuator: tuple[str, np.ndarray, np.ndarray] | None) -> dict[str, float | str] | None:
    """Fraction of samples with a motor on its thrust stop. ``None`` if unlogged."""

    if actuator is None:
        return None
    kind, _time_s, values = actuator
    # A channel that never leaves zero, or is all NaN, is unused.
    column_peak = np.zeros(values.shape[1], dtype=np.float64)
    for index in range(values.shape[1]):
        column = values[:, index]
        column = column[np.isfinite(column)]
        if column.size:
            column_peak[index] = float(np.max(np.abs(column)))
    used = np.isfinite(values) & (column_peak > 1e-3)
    if not np.any(used):
        return {"kind": kind, "fraction": 0.0, "peak": 0.0}
    data = np.where(used, values, np.nan)
    if kind == "pwm":
        on_stop = (data >= PWM_HIGH_US) | (data <= PWM_LOW_US)
        peak = float(np.nanmax(data))
    else:
        on_stop = data >= MOTOR_SATURATION
        peak = float(np.nanmax(data))
    samples = np.any(on_stop, axis=1)
    return {"kind": kind, "fraction": float(np.mean(samples)), "peak": peak}


def _rates(time_s: np.ndarray | None, rates: np.ndarray | None) -> dict[str, float | None] | None:
    """RMS of body rate and the dominant frequency of the strongest axis.

    Frequency comes from the signed axis, not the magnitude. The magnitude
    of a sine is a full-wave rectified signal and would report twice the
    oscillation frequency.
    """

    if time_s is None or rates is None or time_s.shape[0] < 8:
        return None
    rms_axes = [_rms(rates[:, index]) for index in range(3)]
    strongest = int(np.argmax(rms_axes))
    return {
        "rms_x_rad_s": rms_axes[0],
        "rms_y_rad_s": rms_axes[1],
        "rms_z_rad_s": rms_axes[2],
        "rms_rad_s": _rms(np.linalg.norm(rates, axis=1)),
        "dominant_hz": _dominant_hz(time_s, rates[:, strongest]),
    }


def _dominant_hz(time_s: np.ndarray, values: np.ndarray) -> float | None:
    """Frequency of the strongest tone above 0.4 Hz, or ``None`` when it is flat."""

    dt = float(np.median(np.diff(time_s)))
    if dt <= 0.0:
        return None
    centered = values - np.mean(values)
    window = np.hanning(centered.shape[0])
    spectrum = np.abs(np.fft.rfft(centered * window))
    freqs = np.fft.rfftfreq(centered.shape[0], dt)
    spectrum = spectrum.copy()
    spectrum[freqs < 0.4] = 0.0
    if float(np.max(spectrum)) <= 0.0:
        return 0.0
    return float(freqs[int(np.argmax(spectrum))])


def _vision_delay(delay_s: np.ndarray | None) -> dict[str, float] | None:
    """Median, 95th percentile, and max of vision delay. ``None`` if unlogged."""

    if delay_s is None or delay_s.size == 0:
        return None
    finite = delay_s[np.isfinite(delay_s)]
    if finite.size == 0:
        return None
    return {
        "median_s": float(np.median(finite)),
        "p95_s": float(np.percentile(finite, 95)),
        "max_s": float(np.max(finite)),
    }


def _flags(flags: Series | None, t0: float, t1: float) -> dict[str, dict[str, float | bool]] | None:
    """Fraction of the leg each estimator source was on.

    A source counts as active when it is set for at least half the leg, so
    one odd sample does not change the story.
    """

    if flags is None:
        return None
    mask = (flags.time_s >= t0) & (flags.time_s <= t1)
    if not np.any(mask):
        return None
    names = {
        "gps_position": "cs_gnss_pos",
        "vision_position": "cs_ev_pos",
        "vision_yaw": "cs_ev_yaw",
        "baro_height": "cs_baro_hgt",
        "vision_height": "cs_ev_hgt",
        "yaw_aligned": "cs_yaw_align",
    }
    summary: dict[str, dict[str, float | bool]] = {}
    for label, field in names.items():
        if field not in flags.fields:
            continue
        values = np.asarray(flags.fields[field], dtype=np.float64)[mask]
        fraction = float(np.mean(values > 0.5))
        summary[label] = {"fraction": fraction, "active": fraction >= 0.5}
    return summary


def _leak(
    legs: list[Leg],
    estimate: Track,
    ground_truth: Track | None,
    flags: Series | None,
) -> dict[str, object]:
    """Flag a vision hover that matches ground truth to about a millimetre."""

    if ground_truth is None:
        return {"suspect": False, "checked": False, "reason": "no ground truth"}
    windows = _hover_windows(legs, estimate)
    if not windows:
        return {"suspect": False, "checked": False, "reason": "no hover"}
    if flags is None:
        return {
            "suspect": False,
            "checked": False,
            "reason": "estimator_status_flags was not logged",
        }
    worst = 0.0
    saw_vision = False
    for t0, t1 in windows:
        vision = _flag_fraction(flags, "cs_ev_pos", t0, t1)
        if vision is None or vision < 0.5:
            continue
        saw_vision = True
        error = _position_error_peak(estimate, ground_truth, t0, t1)
        if error is None:
            continue
        worst = max(worst, error)
        if error <= VISION_LEAK_M:
            return {
                "suspect": True,
                "checked": True,
                "max_error_m": error,
                "reason": "vision hover matched ground truth within 1 mm",
            }
    if not saw_vision:
        return {"suspect": False, "checked": True, "reason": "not fusing vision position"}
    return {
        "suspect": False,
        "checked": True,
        "max_error_m": worst,
        "reason": "vision hover differed from ground truth",
    }


def _return_distance(estimate: Track, legs: list[Leg]) -> float | None:
    """Horizontal distance from the outbound start to the end of the return.

    Uses the estimate, not the command: the question is where the vehicle
    actually finished, relative to where the first translate began.
    """

    translates = [leg for leg in legs if leg.kind == "translate"]
    if len(translates) < 2 or estimate.time_s.shape[0] == 0:
        return None
    first = translates[0]
    # The leg starts on the first sample of the new command. The position
    # the move left from is the end of the previous leg.
    origin_time = first.t_start_s
    earlier = [leg for leg in legs if leg.t_end_s < first.t_start_s - 1e-9]
    if earlier:
        origin_time = earlier[-1].t_end_s
    start_n, start_e = _nearest_horizontal(estimate, origin_time)
    end_n, end_e = _nearest_horizontal(estimate, translates[-1].t_end_s)
    return float(math.hypot(end_n - start_n, end_e - start_e))


def _nearest_horizontal(track: Track, time_s: float) -> tuple[float, float]:
    """North and east of the sample closest to ``time_s``."""

    index = int(np.argmin(np.abs(track.time_s - time_s)))
    return float(track.north_m[index]), float(track.east_m[index])


def _hover_windows(legs: list[Leg], estimate: Track) -> list[tuple[float, float]]:
    """Steady airborne holds. The post-takeoff hover is the tail of takeoff."""

    windows: list[tuple[float, float]] = []
    for leg in legs:
        if -leg.target_down_m < AIRBORNE_UP_M:
            continue
        if float(leg.t_end_s - leg.t_arrived_s) < 2.0:
            continue
        window = _slice(estimate, leg.t_arrived_s, leg.t_end_s)
        if window.time_s.shape[0] < 2:
            continue
        up = -window.down_m
        speed = np.abs(np.gradient(window.down_m, window.time_s))
        calm = (up >= AIRBORNE_UP_M) & (speed <= HOVER_SPEED_M_S)
        if int(np.count_nonzero(calm)) < 2:
            continue
        times = window.time_s[calm]
        if float(times[-1] - times[0]) >= 2.0:
            windows.append((float(times[0]), float(times[-1])))
    return windows


def _flag_fraction(flags: Series, field: str, t0: float, t1: float) -> float | None:
    """Fraction of samples in the window where a flag is set."""

    if field not in flags.fields:
        return None
    mask = (flags.time_s >= t0) & (flags.time_s <= t1)
    if not np.any(mask):
        return None
    values = np.asarray(flags.fields[field], dtype=np.float64)[mask]
    return float(np.mean(values > 0.5))


def _position_error_peak(
    estimate: Track,
    ground_truth: Track,
    t0: float,
    t1: float,
) -> float | None:
    """Peak 3D distance between the estimate and ground truth on a window."""

    window = _slice(estimate, t0, t1)
    if window.time_s.shape[0] < 2 or ground_truth.time_s.shape[0] < 2:
        return None
    overlap = (window.time_s >= ground_truth.time_s[0]) & (window.time_s <= ground_truth.time_s[-1])
    if not np.any(overlap):
        return None
    time_s = window.time_s[overlap]
    north = np.interp(time_s, ground_truth.time_s, ground_truth.north_m)
    east = np.interp(time_s, ground_truth.time_s, ground_truth.east_m)
    down = np.interp(time_s, ground_truth.time_s, ground_truth.down_m)
    error = np.sqrt(
        (window.north_m[overlap] - north) ** 2
        + (window.east_m[overlap] - east) ** 2
        + (window.down_m[overlap] - down) ** 2
    )
    return float(np.max(error))


def _slice(track: Track, t0: float, t1: float) -> Track:
    """Return the part of ``track`` inside ``[t0, t1]``."""

    mask = (track.time_s >= t0 - 1e-9) & (track.time_s <= t1 + 1e-9)

    def _cut(values: np.ndarray | None) -> np.ndarray | None:
        """Keep the rows of one optional velocity column inside the window."""

        if values is None:
            return None
        return values[mask]

    return Track(
        time_s=track.time_s[mask],
        north_m=track.north_m[mask],
        east_m=track.east_m[mask],
        down_m=track.down_m[mask],
        yaw_rad=track.yaw_rad[mask],
        vn_m_s=_cut(track.vn_m_s),
        ve_m_s=_cut(track.ve_m_s),
        vd_m_s=_cut(track.vd_m_s),
    )


def _wrap_array(angle: np.ndarray) -> np.ndarray:
    """Wrap an array of radians into (-pi, pi]."""

    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return np.where(np.isclose(wrapped, -math.pi), math.pi, wrapped)


def _signed_delta(start: float, target: float) -> float:
    """Shortest yaw change. An exact half-turn stays positive."""

    return wrap_pi(target - start)


def _rms(values: np.ndarray) -> float:
    """Root-mean-square of a 1D sample."""

    if values.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(values))))
