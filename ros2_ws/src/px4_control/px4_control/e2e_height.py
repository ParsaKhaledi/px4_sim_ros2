"""Height check for an end-to-end flight.

Heights are up-positive metres in one frame. The estimator and the setpoint
are NED down, so the caller negates those before passing them in. Ground
truth is the Gazebo ENU z.

The guard is off unless ``E2E_HEIGHT_TOLERANCE_M`` is set. The first sample
past the tolerance is the fault and the abort, so the logged gap is the time
between those two stamps.
"""

from __future__ import annotations


class HeightGuard:
    """Remember the first height fault and the abort that follows it."""

    def __init__(self, tolerance_m: float) -> None:
        tolerance = float(tolerance_m)
        if tolerance <= 0.0 or tolerance != tolerance:
            raise ValueError('E2E_HEIGHT_TOLERANCE_M must be a positive finite number')
        self.tolerance_m = tolerance
        self.fault_time_s: float | None = None
        self.abort_time_s: float | None = None
        self.reason: str | None = None

    def observe(
        self,
        time_s: float,
        estimate_z_up: float,
        setpoint_z_up: float,
        ground_truth_z_up: float,
    ) -> str | None:
        """Return an abort reason on the first sample past the tolerance."""
        if self.abort_time_s is not None:
            return None
        estimate_error = abs(float(estimate_z_up) - float(ground_truth_z_up))
        setpoint_error = abs(float(setpoint_z_up) - float(ground_truth_z_up))
        parts: list[str] = []
        if estimate_error > self.tolerance_m:
            parts.append(f'|estimate z - ground truth z|={estimate_error:.3f} m')
        if setpoint_error > self.tolerance_m:
            parts.append(f'|setpoint z - ground truth z|={setpoint_error:.3f} m')
        if not parts:
            return None
        if self.fault_time_s is None:
            self.fault_time_s = float(time_s)
        self.abort_time_s = float(time_s)
        self.reason = (
            f'height tolerance {self.tolerance_m:.3f} m exceeded: ' + '; '.join(parts)
        )
        return self.reason

    @property
    def fault_to_abort_s(self) -> float | None:
        if self.fault_time_s is None or self.abort_time_s is None:
            return None
        return self.abort_time_s - self.fault_time_s


def height_abort_log(reason: str, fault_to_abort_s: float) -> str:
    """One line for the flight log: the cause and the fault-to-abort gap."""
    return f'height abort: {reason}; fault_to_abort_s={float(fault_to_abort_s):.3f}'
