# Flight mechanics and control

Plan and status for flight dynamics, control review, and flight-log grading. Status as of 2026-10-09 03:20 Tehran time.

Nothing is running: no UFO processes, no jobs on the shared machine, and no active cloud runs.

## Done

- **PR #26** (`feat/flight-analysis` → `dev/px4-upgrade`, draft), head `f1cd670`, 42 unit tests on synthetic signals. It is an offline grader for PX4 `.ulg` logs. It reports overshoot, settling time, and stopping distance per leg; tracking error separate from estimation error; tilt; saturation; body-rate oscillation; height divergence; estimator flags; vision delay; and the ground-truth-leak check. Limits come only from the environment, then `.env`, using #19's 21 `E2E_*` names. The spawn pose and clock come from `spawn.json`, with CLI overrides.
- **Review of #22** (`px4_control`). Five flight-blocking bugs were posted: an acceleration spike at the end of moves, `turn(+180)` going clockwise, the `cmd_vel` timeout counted twice, abort freezing the setpoint, and settle tolerances 10× too loose. The PX4 expert reports the fixes are pushed on `feat/px4-control` at `7e7887f`.
- **Design inputs taken into #22.** The wall frame is computed at arming from the spawn transform. Height is checked independently as lidar against EKF2. Vision is taken out of height (`EV_CTRL=9`, barometer height reference, range aiding).

## Status

| Item | Where the work is | Where it stopped | Waiting on |
| --- | --- | --- | --- |
| Run #26 on a real flight log | `feat/flight-analysis` @ `f1cd670` | Stopped before the first real run. No real `.ulg` exists yet: the UFO run had no flights, and #19's failed CI flight log was not saved. Unverified: pyulog field names on a v1.17 SITL log, the PX4/ROS clock offset, and how the plots look. | The first saved `.ulg` plus ground truth, and #19's `E2E_*` keys in `.env` |
| `spawn.json` input | The reader is done in #26. The producer is `trajectory_eval record` on #21. | The producer was cancelled before any code was pushed. Until it comes back, pass `--spawn-xyz`, `--spawn-yaw`, and `--clock-offset` by hand from the world's spawn settings. | #21 |
| Re-review the #22 fixes for bugs 1–5 and their regression tests, plus the per-action files and loop rates | `feat/px4-control` @ `7e7887f` | Not started | Nothing. Ready to start. |
| Frames and control section for the main README (Gazebo ENU, ROS ENU/FLU, PX4 NED/FRD with a diagram, the command flow through PX4's control loops, and the reasoning behind the limits) | Not started | Not started | #19's `docs/templates/FOLDER_README.md` |
| Stopping-distance e2e test | Not started | Not started | #22's fixes merging, and #19 |
| Controller metrics on `feat/observability` (will reuse #26's metrics) | Not started | Not started | #19 |
| UFO flight turn | Not started | Not started | #22's fixes and a free UFO slot |

## Next steps

1. Re-review #22 at `7e7887f`.
2. Run #26 on the first real log and fix any field names.
3. Write the Frames and control docs.
4. The stopping-distance test.
5. Controller metrics on `feat/observability`.
6. The UFO flight.

## Usage

Stop at 88% of the usage budget and wait for Parsa. Never pass 95%. Use grok-4.7 only, up to 70% of its usage.
