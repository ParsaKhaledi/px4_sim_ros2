"""Known step responses: overshoot, settling, yaw, and a height divergence."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
for entry in (str(ROOT), str(HERE)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

from flight_analysis.analyze import analyze_flight  # noqa: E402
from flight_analysis.log import command_track  # noqa: E402
from flight_analysis.log import estimate_track  # noqa: E402
from flight_analysis.metrics import settling_time_s  # noqa: E402
from flight_analysis.segment import segment_command  # noqa: E402
from synthetic import flight_log  # noqa: E402
from synthetic import level_quaternion  # noqa: E402
from synthetic import load_fixture_limits  # noqa: E402
from synthetic import microseconds  # noqa: E402
from synthetic import times  # noqa: E402


def _checks(report: dict, name: str, kind: str | None = None) -> list[dict]:
    """Checks with this name, optionally limited to one leg kind."""

    rows = [check for check in report["checks"] if check["name"] == name]
    if kind is None:
        return rows
    return [check for check in rows if check["kind"] == kind]


def _position_topics(
    time_s: np.ndarray,
    command_n: np.ndarray,
    estimate_n: np.ndarray,
    command_d: np.ndarray | None = None,
    estimate_d: np.ndarray | None = None,
    command_yaw: np.ndarray | None = None,
    estimate_yaw: np.ndarray | None = None,
    truth_d: np.ndarray | None = None,
    truth_n: np.ndarray | None = None,
) -> dict:
    """Setpoint and local-position topics for a north/down manoeuvre."""

    count = time_s.shape[0]
    zeros = np.zeros(count)
    command_d = zeros if command_d is None else command_d
    estimate_d = zeros if estimate_d is None else estimate_d
    command_yaw = zeros if command_yaw is None else command_yaw
    estimate_yaw = zeros if estimate_yaw is None else estimate_yaw
    stamp = microseconds(time_s)
    topics = {
        "trajectory_setpoint": {
            "timestamp": stamp,
            "position[0]": command_n,
            "position[1]": zeros,
            "position[2]": command_d,
            "yaw": command_yaw,
        },
        "vehicle_local_position": {
            "timestamp": stamp,
            "x": estimate_n,
            "y": zeros,
            "z": estimate_d,
            "heading": estimate_yaw,
        },
    }
    if truth_d is not None:
        topics["vehicle_local_position_groundtruth"] = {
            "timestamp": stamp,
            "x": estimate_n if truth_n is None else truth_n,
            "y": zeros,
            "z": truth_d,
            "heading": estimate_yaw,
        }
    return topics


class StepResponseTests(unittest.TestCase):
    """Numbers you can compute by hand from the fixture limits."""

    def setUp(self) -> None:
        """Load pass/fail limits from the fixture env file."""

        self.limits = load_fixture_limits()

    def test_position_overshoot_and_settling_time(self) -> None:
        """A step that peaks past the target, then sits on it, has both numbers."""

        limits = self.limits
        late = limits.settle_hold_s * 0.5
        self.assertLessEqual(late + limits.settle_hold_s, limits.settle_timeout_s)
        # Midway between the settle band and the overshoot limit: outside the
        # band, so settling cannot start, and still a passing overshoot.
        self.assertGreater(limits.overshoot_m, limits.settle_tolerance_m)
        extra = 0.5 * (limits.overshoot_m + limits.settle_tolerance_m)
        target = 1.0
        t0 = 1.0
        time_s = times(0.0, t0 + limits.settle_timeout_s + limits.settle_hold_s)
        command = np.where(time_s >= t0, target, 0.0)
        enter = t0 + late
        estimate = np.where(time_s < t0, 0.0, np.where(time_s < enter, target + extra, target))
        report = analyze_flight(flight_log(_position_topics(time_s, command, estimate)), limits)
        overshoot = _checks(report, "overshoot", "translate")
        self.assertTrue(overshoot)
        self.assertTrue(overshoot[0]["passed"])
        self.assertAlmostEqual(overshoot[0]["measured"], extra, delta=1e-6)
        settle = _checks(report, "settle", "translate")
        self.assertTrue(settle[0]["passed"], settle[0]["reason"])
        self.assertAlmostEqual(settle[0]["measured"], late + limits.settle_hold_s, delta=0.06)

    def test_overshoot_past_the_limit_fails(self) -> None:
        """Staying beyond the overshoot limit fails that check."""

        limits = self.limits
        extra = limits.overshoot_m + limits.settle_tolerance_m
        time_s = times(0.0, 8.0)
        command = np.where(time_s >= 1.0, 1.0, 0.0)
        estimate = np.where(time_s >= 1.0, 1.0 + extra, 0.0)
        report = analyze_flight(flight_log(_position_topics(time_s, command, estimate)), limits)
        overshoot = _checks(report, "overshoot", "translate")[0]
        self.assertFalse(overshoot["passed"])
        self.assertAlmostEqual(overshoot["measured"], extra, delta=1e-6)
        self.assertFalse(report["passed"])

    def test_stopping_distance_is_where_it_comes_to_rest(self) -> None:
        """Going past the target and returning leaves overshoot, not stopping distance."""

        limits = self.limits
        extra = limits.overshoot_m * 0.5
        time_s = times(0.0, 8.0)
        command = np.where(time_s >= 1.0, 1.0, 0.0)
        estimate = np.zeros_like(time_s)
        for index, stamp in enumerate(time_s):
            if stamp < 1.0:
                estimate[index] = 0.0
            elif stamp < 1.3:
                estimate[index] = (1.0 + extra) * (stamp - 1.0) / 0.3
            elif stamp < 1.6:
                estimate[index] = (1.0 + extra) + (1.0 - (1.0 + extra)) * (stamp - 1.3) / 0.3
            else:
                estimate[index] = 1.0
        report = analyze_flight(flight_log(_position_topics(time_s, command, estimate)), limits)
        overshoot = _checks(report, "overshoot", "translate")[0]
        stopping = _checks(report, "stopping_distance", "translate")[0]
        self.assertAlmostEqual(overshoot["measured"], extra, delta=1e-6)
        self.assertAlmostEqual(stopping["measured"], 0.0, delta=1e-6)

        stayed = np.where(time_s >= 1.0, 1.0 + extra, 0.0)
        stayed_report = analyze_flight(
            flight_log(_position_topics(time_s, command, stayed)), limits
        )
        stayed_stop = _checks(stayed_report, "stopping_distance", "translate")[0]
        self.assertAlmostEqual(stayed_stop["measured"], extra, delta=1e-6)

    def test_yaw_half_turn_overshoot(self) -> None:
        """A 180 deg yaw that goes past the new heading by a known angle."""

        limits = self.limits
        extra_deg = limits.yaw_tolerance_deg * 0.5
        time_s = times(0.0, 8.0)
        command_yaw = np.where(time_s >= 1.0, math.pi, 0.0)
        estimate_yaw = np.zeros_like(time_s)
        enter = 1.0 + limits.settle_hold_s * 0.5
        for index, stamp in enumerate(time_s):
            if stamp < 1.0:
                estimate_yaw[index] = 0.0
            elif stamp < enter:
                estimate_yaw[index] = math.pi + math.radians(extra_deg)
            else:
                estimate_yaw[index] = math.pi
        zeros = np.zeros_like(time_s)
        report = analyze_flight(
            flight_log(
                _position_topics(
                    time_s,
                    zeros,
                    zeros,
                    command_yaw=command_yaw,
                    estimate_yaw=estimate_yaw,
                )
            ),
            limits,
        )
        overshoot = _checks(report, "yaw_overshoot", "yaw")[0]
        self.assertTrue(overshoot["passed"], overshoot)
        self.assertAlmostEqual(overshoot["measured"], extra_deg, delta=0.05)
        too_far = limits.yaw_tolerance_deg + limits.yaw_settle_deg
        estimate_yaw = np.where(time_s >= 1.0, math.pi + math.radians(too_far), 0.0)
        failed = analyze_flight(
            flight_log(
                _position_topics(
                    time_s,
                    zeros,
                    zeros,
                    command_yaw=command_yaw,
                    estimate_yaw=estimate_yaw,
                )
            ),
            limits,
        )
        bad = _checks(failed, "yaw_overshoot", "yaw")[0]
        self.assertFalse(bad["passed"])
        self.assertGreater(bad["measured"], limits.yaw_tolerance_deg)

    def test_height_divergence_flags_the_first_crossing(self) -> None:
        """Estimate holds the setpoint while ground truth climbs through the limit."""

        limits = self.limits
        time_s = times(0.0, 12.0)
        estimate_d = np.full_like(time_s, -2.0)
        truth_up = np.full_like(time_s, 2.0)
        ramp = (time_s >= 8.0) & (time_s <= 10.0)
        truth_up[ramp] = 2.0 + 2.0 * limits.height_tolerance_m * (time_s[ramp] - 8.0) / 2.0
        truth_up[time_s > 10.0] = 2.0 + 2.0 * limits.height_tolerance_m
        zeros = np.zeros_like(time_s)
        report = analyze_flight(
            flight_log(
                _position_topics(
                    time_s,
                    zeros,
                    zeros,
                    command_d=estimate_d,
                    estimate_d=estimate_d,
                    truth_d=-truth_up,
                )
            ),
            limits,
        )
        check = _checks(report, "height_divergence")[0]
        self.assertFalse(check["passed"], check)
        self.assertGreater(check["measured"], limits.height_tolerance_m)
        event = report["flight"]["height_divergence"]
        self.assertAlmostEqual(event["time_s"], 9.0, delta=0.06)
        self.assertAlmostEqual(event["estimate_up_m"], 2.0, places=6)
        self.assertEqual(report["ground_truth"]["source"], "ulog")

    def test_missing_ground_truth_skips_estimation(self) -> None:
        """No truth topic is a skip, not a failed flight."""

        time_s = times(0.0, 4.0)
        zeros = np.zeros_like(time_s)
        report = analyze_flight(flight_log(_position_topics(time_s, zeros, zeros)), self.limits)
        self.assertEqual(report["ground_truth"]["source"], "none")
        self.assertIsNone(report["flight"]["estimation_error"])
        divergence = _checks(report, "height_divergence")[0]
        self.assertTrue(divergence["skipped"])
        self.assertTrue(divergence["passed"])

    def test_timestamps_are_sim_microseconds(self) -> None:
        """A 1-second ULog stamp becomes 1.0 s inside the tool."""

        stamp = np.array([1_000_000, 2_000_000])
        log = flight_log(
            {
                "trajectory_setpoint": {
                    "timestamp": stamp,
                    "position[0]": np.array([0.0, 1.0]),
                    "position[1]": np.array([0.0, 0.0]),
                    "position[2]": np.array([0.0, 0.0]),
                    "yaw": np.array([0.0, 0.0]),
                },
                "vehicle_local_position": {
                    "timestamp": stamp,
                    "x": np.array([0.0, 1.0]),
                    "y": np.array([0.0, 0.0]),
                    "z": np.array([0.0, 0.0]),
                    "heading": np.array([0.0, 0.0]),
                },
            }
        )
        self.assertAlmostEqual(float(command_track(log).time_s[0]), 1.0)
        self.assertAlmostEqual(float(estimate_track(log).time_s[1]), 2.0)

    def test_nan_setpoint_holds_the_previous_command(self) -> None:
        """NaN in a setpoint axis means that axis is left where it was."""

        stamp = microseconds(np.array([0.0, 0.1, 0.2]))
        log = flight_log(
            {
                "trajectory_setpoint": {
                    "timestamp": stamp,
                    "position[0]": np.array([1.0, np.nan, np.nan]),
                    "position[1]": np.array([0.0, 0.0, 0.0]),
                    "position[2]": np.array([0.0, 0.0, 0.0]),
                    "yaw": np.array([0.0, 0.0, 0.0]),
                },
                "vehicle_local_position": {
                    "timestamp": stamp,
                    "x": np.array([1.0, 1.0, 1.0]),
                    "y": np.zeros(3),
                    "z": np.zeros(3),
                    "heading": np.zeros(3),
                },
            }
        )
        command = command_track(log)
        self.assertTrue(np.allclose(command.north_m, 1.0))

    def test_ramp_is_one_leg(self) -> None:
        """A slide into a hold is one translate, not a leg per sample."""

        time_s = times(0.0, 6.0)
        command = np.zeros_like(time_s)
        slide = (time_s >= 1.0) & (time_s < 3.0)
        command[slide] = 0.3 * (time_s[slide] - 1.0) / 2.0
        command[time_s >= 3.0] = 0.3
        log = flight_log(_position_topics(time_s, command, command))
        kinds = [leg.kind for leg in segment_command(command_track(log))]
        self.assertEqual(kinds.count("translate"), 1)

    def test_body_rate_frequency_and_vision_delay(self) -> None:
        """A 5 Hz sine and a known vision delay show up in the flight summary."""

        time_s = times(0.0, 8.0, dt=0.01)
        zeros = np.zeros_like(time_s)
        rate = 0.2 * np.sin(2.0 * math.pi * 5.0 * time_s)
        delay_s = np.array([0.01, 0.02, 0.03, 0.04, 0.10])
        vision_time = np.arange(delay_s.shape[0], dtype=np.float64)
        topics = _position_topics(time_s, zeros, zeros)
        topics["vehicle_angular_velocity"] = {
            "timestamp": microseconds(time_s),
            "xyz[0]": rate,
            "xyz[1]": zeros,
            "xyz[2]": zeros,
        }
        topics["vehicle_visual_odometry"] = {
            "timestamp": microseconds(vision_time),
            "timestamp_sample": microseconds(vision_time - delay_s),
        }
        report = analyze_flight(flight_log(topics), self.limits)
        rates = report["flight"]["body_rates"]
        self.assertAlmostEqual(rates["dominant_hz"], 5.0, delta=0.2)
        self.assertAlmostEqual(rates["rms_rad_s"], 0.2 / math.sqrt(2.0), delta=0.01)
        delay = report["flight"]["vision_delay"]
        self.assertAlmostEqual(delay["median_s"], float(np.median(delay_s)), places=6)
        self.assertAlmostEqual(delay["max_s"], 0.10, places=6)
        self.assertAlmostEqual(delay["p95_s"], float(np.percentile(delay_s, 95)), places=6)

    def test_tilt_limit_and_saturation(self) -> None:
        """Tilt is compared with MPC_TILTMAX_AIR, and a full motor fails."""

        time_s = times(0.0, 2.0)
        zeros = np.zeros_like(time_s)
        topics = _position_topics(time_s, zeros, zeros)
        attitude = level_quaternion(time_s.shape[0], math.radians(50.0))
        attitude["timestamp"] = microseconds(time_s)
        topics["vehicle_attitude"] = attitude
        topics["actuator_motors"] = {
            "timestamp": microseconds(time_s),
            "control[0]": np.full(time_s.shape, 1.0),
            "control[1]": np.full(time_s.shape, np.nan),
        }
        report = analyze_flight(
            flight_log(topics, {"MPC_TILTMAX_AIR": 45.0}),
            self.limits,
        )
        tilt = _checks(report, "tilt")[0]
        self.assertFalse(tilt["passed"])
        self.assertAlmostEqual(tilt["measured"], 50.0, delta=0.1)
        self.assertAlmostEqual(tilt["limit"], 45.0)
        saturation = _checks(report, "actuator_saturation")[0]
        self.assertFalse(saturation["passed"])
        self.assertGreater(saturation["measured"], 0.0)

    def test_vision_hover_matching_truth_is_suspect(self) -> None:
        """A vision hover inside 1 mm of truth is flagged; a  centimetre gap is not."""

        limits = self.limits
        phase = limits.settle_hold_s * 3
        time_s = times(0.0, phase * 2)
        down = np.where(time_s >= phase, -2.0, 0.0)
        zeros = np.zeros_like(time_s)
        flags = np.where(time_s >= phase, 1.0, 0.0)
        topics = _position_topics(
            time_s,
            zeros,
            zeros,
            command_d=down,
            estimate_d=down,
            truth_d=down,
        )
        topics["estimator_status_flags"] = {
            "timestamp": microseconds(time_s),
            "cs_ev_pos": flags,
            "cs_ev_yaw": flags,
            "cs_gnss_pos": np.zeros_like(time_s),
            "cs_baro_hgt": np.ones_like(time_s),
            "cs_ev_hgt": flags,
            "cs_yaw_align": np.ones_like(time_s),
        }
        suspect = analyze_flight(flight_log(topics), limits)
        leak = _checks(suspect, "vision_ground_truth_leak")[0]
        self.assertFalse(leak["passed"], leak)
        self.assertFalse(leak["skipped"])

        topics["vehicle_local_position_groundtruth"]["z"] = down - limits.settle_tolerance_m
        fine = analyze_flight(flight_log(topics), limits)
        ok = _checks(fine, "vision_ground_truth_leak")[0]
        self.assertTrue(ok["passed"], ok)
        takeoff = next(leg for leg in fine["legs"] if leg["kind"] == "takeoff")
        self.assertTrue(takeoff["estimator_flags"]["vision_position"]["active"])
        self.assertFalse(takeoff["estimator_flags"]["gps_position"]["active"])


class MeasureDirectTests(unittest.TestCase):
    """The settle helper's deadline, without the rest of the flight."""

    def test_settle_that_finishes_after_the_timeout_is_none(self) -> None:
        """Entering the band too late to finish the hold returns no settle time."""

        limits = load_fixture_limits()
        time_s = times(0.0, limits.settle_timeout_s + limits.settle_hold_s + 1.0)
        error = np.where(time_s >= limits.settle_timeout_s, 0.0, limits.settle_tolerance_m + 1.0)
        self.assertIsNone(
            settling_time_s(
                time_s,
                error,
                0.0,
                limits.settle_tolerance_m,
                limits.settle_hold_s,
                limits.settle_timeout_s,
            )
        )


if __name__ == "__main__":
    unittest.main()
