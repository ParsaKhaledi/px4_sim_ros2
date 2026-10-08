# Simulation plan (`feat/sim-headless-groundtruth`, PR #21)

Status on Oct 9, 2026, 03:18 Tehran: draft, unmerged, head `d8d8d03`, 66 tests pass. Work paused; resumes on Parsa's word.

## Done
- Headless Gazebo (EGL/Xvfb, software GL), real-time factor topic, sim time in Nav2, bridge and state publisher.
- `/ground_truth/odom` (ENU, 50 Hz) with its covariance topic moved off `/model/<model>/odometry_with_covariance`, so ground truth never reaches EKF2.
- Static `world` → `spawn` transform and geographic spawn at the Amirkabir Aerospace Department.
- Sensor systems loaded from the x500 model; downward lidar on `x500_depth` (`LIDAR_DOWN`).
- Offline worlds and wall maps (`includes/gz/walls/<world>.json`, world ENU).
- `trajectory_eval` (TUM, ATE/RPE, GPS and EKF2 against ground truth).
- `/sim/preflight_check`: stale topics fail, camera checked through `camera_info`, steady-clock discovery, IMU minimum 0.75 of the expected rate.
- Docstrings and folder READMEs.

## Where each item was when work stopped
| Item | Where | Stopped at |
| --- | --- | --- |
| `spawn.json` from `trajectory_eval record` | this branch | Run cancelled at 03:18 before any code was pushed. Key names agreed with #26; nothing written yet. |
| UFO live check with #20 | Parsa's laptop UFO, separate worktree | Not started; waiting for DevOps' #19 run to free UFO. |
| Rebase onto #19 and #20 | this branch | Not started; #19 and #20 unmerged. Vision README lines from #20 are kept for it. |
| Cleanup PRs | new branches from this one | Read-only review done; the four preflight bugs it found are fixed (`c8de8af`, `20c869a`, `d8d8d03`). Cleanup PR 1 not started. |
| Mission benchmark | after #22 | Not started; waits for #22. |
| Demo GIF capture | with DevOps' `make demo-gif` | Not started; waits for #19 to #22. |

## Next, in order
1. `trajectory_eval record` writes `spawn.json`: `spawn_xyz`, `spawn_yaw` (ENU, from the live `world` → `spawn` transform), `px4_offset_s` = t_ros − t_px4 (lowest gap between `/fmu/out/vehicle_odometry` and `/clock` over at least 50 messages), `px4_offset_spread_s`, `px4_offset_samples`, `px4_offset_reason`. Read by the flight-analysis tool (#26).
2. Live CPU check on UFO with #20: preflight passes, no publisher on `/model/.../odometry_with_covariance`, lidar works, real-time factor with `LIDAR_DOWN=1` and `0`. Separate worktree and compose project, cleaned up afterwards.
3. Rebase after #19 (drop the duplicate start-script fix) and #20 (one `/imu` publisher, bridged only with `IMU_SOURCE=oak`; add the Vision section to `includes/gz/startFiles/README.md`).
4. Cleanup PRs, one at a time: golden tests and one pytest command; dead code; shared PX4-path and SDF helpers; one defaults file and one spawn-pose parser; preflight speed; shared ROS math; Python 3.12 tidy; start-script and compose merge (after #19); sensor rate tables (after #20); trajectory_eval stamping.
5. Mission benchmark (after #22): out-and-back, 3 runs, median and spread of sim-time tick jitter, setpoint error against ground truth, minimum wall distance; runs below real-time factor 0.4 flagged, not graded.
6. Demo GIF capture: Gazebo and RViz (RTAB-Map) side by side under Xvfb, sped up by the logged real-time factor, under 8 MB at `docs/media/demo.gif`.

## Rules
PX4 v1.17.0, CPU-only for now, commits authored by Parsa with no trailers, nothing merged without Parsa.
