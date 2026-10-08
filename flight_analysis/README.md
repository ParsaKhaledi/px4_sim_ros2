# Flight analysis

Offline grader for a PX4 v1.17 SITL flight. It reads a `.ulg` log, splits the flight into legs from the commanded setpoint, and writes control metrics plus plots. It does not start ROS or Docker, so it can run after a flight on a laptop or in CI.

The out-and-back it is aimed at is: take off, hold, fly forward, yaw 180°, fly forward back to the start, and land.

## Run

From the repository root, with Python 3.12:

```bash
python3 -m pip install -r flight_analysis/requirements.txt
python3 -m flight_analysis path/to/flight.ulg
python3 -m flight_analysis logs/<run_id>/flight.ulg \
    --ground-truth logs/<run_id>/ground_truth.tum --run-id demo
```

`--output` defaults to `logs/<run_id>/control/`. `--run-id` defaults to the log file name without `.ulg`.

Exit codes: `0` when every graded check passes, `1` when a check fails (the report is still written), `2` when the log, the ground-truth overlap, the spawn pose, or a limit cannot be read.

The three packages in `requirements.txt` (`pyulog`, `numpy`, `matplotlib`) are for this tool only. They are not added to the simulation image.

### Tests

```bash
python3 -m unittest discover -s tests/flight_analysis -v
```

The tests build signals in memory. They do not need a real `.ulg`. Limit numbers come from `tests/flight_analysis/fixtures/e2e.env`, not from literals in the test code.

## Inputs

| Input | What it is |
| --- | --- |
| `.ulg` | PX4 flight log. Topic timestamps are sim-time microseconds. The tool converts those to seconds at the loader boundary. |
| TUM file, optional | `timestamp tx ty tz qx qy qz qw`, one pose per line, seconds of ROS sim time. Position is Gazebo world ENU (x east, y north, z up) and the quaternion is body FLU. |
| `spawn.json`, with a TUM file | Beside the `.ulg` (`logs/<run_id>/spawn.json`), or `--spawn-json PATH`. |

`trajectory_setpoint` is the command. If that topic is missing, `vehicle_local_position_setpoint` is used. The estimate is EKF2 `vehicle_local_position`. Tilt comes from `vehicle_attitude`. Body rates come from `vehicle_angular_velocity`. Motor stops come from `actuator_motors` (0 idle, 1 full) or, if that is absent, `actuator_outputs` PWM.

### Ground truth

A TUM file is preferred over `vehicle_local_position_groundtruth` and `vehicle_attitude_groundtruth`. `metrics.json` records which source was used. If neither is present, estimation error and the height check are skipped and the run does not fail for that reason.

The TUM poses are Gazebo world ENU. Before they are compared with the PX4 log they are moved into the takeoff frame with the spawn pose, then converted from ENU/FLU to NED/FRD. The spawn file looks like this:

```json
{
  "spawn_xyz": [1.0, 2.0, 0.1],
  "spawn_yaw": 0.3,
  "px4_offset_s": 12.5,
  "px4_offset_spread_s": 0.004,
  "px4_offset_samples": 120
}
```

`spawn_xyz` is `[x, y, z]` in metres, Gazebo world ENU. `spawn_yaw` is the spawn heading in that frame, radians, 0 facing east and positive toward north. The world pose is the spawn pose composed with the local pose: `p_world = R_z(spawn_yaw) p_local + spawn_xyz`. The tool applies the inverse, then `frames.enu_flu_to_ned_frd`. North is the local ENU y, east is the local ENU x, down is the negated local ENU z. Yaw follows `yaw_ned = π/2 - yaw_enu` (0 is north, positive toward east).

`--spawn-xyz X Y Z` and `--spawn-yaw RAD` override the matching file fields. If a ground-truth file is given and neither the file nor those options supply a complete pose, the command stops. It does not assume the vehicle spawned at the origin, because that would shift every error.

`px4_offset_s` is ROS sim time minus PX4 boot time, in seconds: `t_px4 = t_ros - px4_offset_s`. The seconds added to each TUM timestamp are therefore `-px4_offset_s`. `--clock-offset` is the same quantity and wins over the file. If the key is missing, or it is JSON `null`, the tool estimates the offset from the climb (up-velocity) and prints a warning. A null offset is what `trajectory_eval record` writes when PX4 was not publishing. The warning includes `px4_offset_reason` when that string is in the file. The run does not stop for a null offset.

`px4_offset_spread_s` is the median gap minus the minimum gap, in seconds. `px4_offset_samples` is the number of samples in that estimate. When either key is present it is copied into `metrics.json` next to the offset. A spread over 0.02 s, or fewer than 50 samples, prints a warning; the offset is still used. Any other key in the file is ignored.

`metrics.json` records `spawn.xyz_source`, `spawn.yaw_source`, and `clock.source` as `spawn_json`, `cli`, or, for the clock only, `climb_edge_estimate`, plus the values that were used. The overlap after the shift must cover at least a few seconds. An estimated clock also has to correlate with the climb; an explicit clock does not, but a short overlap is still an error.

## Outputs

Everything goes in the output directory.

| File | Contents |
| --- | --- |
| `metrics.json` | Checks, per-leg numbers, and the ground-truth source. |
| `position.png` | North, east, and up: setpoint, EKF2 estimate, and ground truth when present. |
| `yaw.png` | Unwrapped yaw for the same three traces. |
| `tilt.png` | Tilt off vertical, and `MPC_TILTMAX_AIR` when that parameter was logged. |
| `errors.png` | Horizontal and height error. Tracking is estimate versus setpoint. Estimation is estimate versus ground truth. |

### `metrics.json`

Top-level fields:

| Field | Meaning |
| --- | --- |
| `run_id`, `log` | Name and path of the flight. |
| `time_base` | `sim_seconds`. |
| `passed` | True when every check that was not skipped passed. |
| `ground_truth` | `source` is `tum`, `ulog`, or `none`. For a TUM file, `time_offset_s`, `correlation`, `alignment`, and `overlap_s` say how the clocks were matched. |
| `limits` | The E2E values used for this run. |
| `checks` | One object per check: `name`, `scope` (`flight` or `leg`), `leg`, `kind`, `passed`, `skipped`, `measured`, `limit`, `reason`. A skipped check is not a failure. |
| `flight` | Whole-flight tracking and estimation error, tilt, actuator saturation, body rates, the first height divergence, vision delay, the leak flag, and return distance. |
| `legs` | One object per commanded target. |

Each leg has `kind` (`hold`, `takeoff`, `hover`, `translate`, `yaw`, `land`), `t_start_s`, `t_arrived_s` (when the command reached the new target), `t_end_s`, the target in NED, overshoot, settling time, stopping distance, tracking error, estimation error, and `estimator_flags`.

Error blocks report RMS and peak horizontal error, vertical error, and yaw error. Tracking compares the setpoint with `vehicle_local_position`. Estimation compares that estimate with ground truth.

`estimator_flags` is the fraction of the leg each source was on: GPS position (`cs_gnss_pos`), vision position (`cs_ev_pos`), vision yaw (`cs_ev_yaw`), baro height (`cs_baro_hgt`), vision height (`cs_ev_hgt`), and yaw alignment (`cs_yaw_align`). `active` means the flag was set for at least half the leg.

## Legs

A leg is one commanded target. It starts when `trajectory_setpoint` moves faster than a small deadband and lasts until the next change, including the hold after the move. NaN in a setpoint component means "leave this axis alone", so the previous command is kept. A 0.30 m slide is one translate, not a leg per sample. The 10 s hover after takeoff is the steady tail of the takeoff leg unless the setpoint changes again.

## Checks

| Check | Setting | What it means |
| --- | --- | --- |
| `overshoot` | `E2E_OVERSHOOT_M` | Peak distance the estimate travels past the position target, along the commanded move. The flight-scoped check is the worst leg. |
| `yaw_overshoot` | `E2E_YAW_TOLERANCE_DEG` | Peak yaw past the heading target, in the direction of the command. An exact 180° turn is kept positive so the branch cut does not flip the sign. |
| `settle` | `E2E_SETTLE_TOLERANCE_M`, `E2E_SETTLE_HOLD_S`, `E2E_SETTLE_TIMEOUT_S` | Time from when the command reaches the target until the estimate has stayed inside the position band for the hold. The hold must finish before the timeout, so the reported time includes the hold. |
| `yaw_settle` | `E2E_YAW_SETTLE_DEG`, `E2E_SETTLE_HOLD_S`, `E2E_SETTLE_TIMEOUT_S` | Same clock for yaw, using the yaw band. |
| `stopping_distance` | `E2E_OVERSHOOT_M` | How far past the target the vehicle still is once horizontal speed stays near zero. Overshoot can be larger if it goes past and comes back. |
| `return_to_start` | `E2E_RETURN_TOLERANCE_M` | Horizontal distance from where the first translate began to where the last translate ended. Skipped when the log is not an out-and-back. |
| `height_divergence` | `E2E_HEIGHT_TOLERANCE_M` | First time the estimate height and ground-truth height disagree by more than the limit. This is the failure where the estimate sits on the setpoint while the vehicle climbs. |
| `tilt` | logged `MPC_TILTMAX_AIR` | Peak tilt off vertical against that parameter, in degrees. Skipped when attitude or the parameter is missing. |
| `actuator_saturation` | motors on the stop | Any sample with a used motor at or above 95% thrust, or PWM within 100 µs of the usual 1000/2000 µs rails. Skipped when neither topic is logged. |
| `body_rates` | none | RMS of body rate and the dominant frequency of the strongest signed axis. There is no E2E oscillation limit; the check records the numbers. |
| `vision_delay` | none | `timestamp` minus `timestamp_sample` on `vehicle_visual_odometry`: median, 95th percentile, and max. |
| `vision_ground_truth_leak` | about 1 mm | If vision position is fused and, for a whole airborne hover, the estimate stays within about 1 mm of ground truth, the run is marked suspect. A real vision fix wanders by centimetres. |
| `estimator_flags` | none | Present when `estimator_status_flags` is in the log. The per-leg fractions are the detail. |

These names are also loaded so the in-flight grader can share one module, but this tool does not grade against them: `E2E_TAKEOFF_HEIGHT_M`, `E2E_HOVER_S`, `E2E_LEG_LENGTH_M`, `E2E_HOVER_DRIFT_M`, `E2E_LEG_TOLERANCE_M`, `E2E_HOVER_HEIGHT_BAND_M`, `E2E_HOVER_HEIGHT_HOLD_S`, `E2E_MAX_RETRIES`, `E2E_CRASH_TILT_DEG`, `E2E_CRASH_MIN_HEIGHT_M`, `E2E_CRASH_IMPACT_SPEED_MPS`, `E2E_CRASH_DIVERGENCE_M`, `E2E_ODOM_TIMEOUT_S`.

## Where the numbers come from

`e2e_limits.py` (repository root) lists the limit names and parses them. It does not store default numbers. For each name it uses the process environment, then `.env` at the repo root. If a name is missing from both, the tool exits and names every missing key. `.env.example` is not read as a fallback. When both files exist, a test requires their `E2E_*` keys and values to match. A separate test fails if a value is assigned outside those two files and the test fixture.

This branch's `.env` and `.env.example` do not contain those keys yet. Until they land, put them in the environment or in `.env`.

## Files

| File | Role |
| --- | --- |
| `e2e_limits.py` | Shared limit names, env-file parsing, and validation. Standard library only. |
| `flight_analysis/__main__.py` | `python -m flight_analysis`. |
| `flight_analysis/cli.py` | Arguments and exit codes. |
| `flight_analysis/analyze.py` | Wires load, segment, measure, grade, and output. |
| `flight_analysis/log.py` | `FlightLog` and the pyulog loader. Tests build the same objects from arrays. |
| `flight_analysis/segment.py` | Legs from setpoint changes. |
| `flight_analysis/metrics.py` | Overshoot, settling, stopping distance, errors, tilt, rates, height, flags, vision delay, leak check. |
| `flight_analysis/grade.py` | Pass/fail against the loaded limits. |
| `flight_analysis/plots.py` | The four PNGs. |
| `flight_analysis/frames.py` | Spawn-frame transform, then ENU/FLU to NED/FRD. |
| `flight_analysis/spawn.py` | `spawn.json` and the CLI pose and clock overrides. |
| `flight_analysis/tum.py` | TUM reader and clock alignment. |
| `flight_analysis/requirements.txt` | Pinned `pyulog`, `numpy`, and `matplotlib`. |
