# px4_control plan and status (PR #22)

Branch: `feat/px4-control` into `dev/px4-upgrade` (draft, not merged).
Head when work stopped: `7e7887f` (Oct 9 2026, 03:18 Tehran). All commits are authored by Parsa Khaledi.
Work stopped on Parsa's request (weekly usage limit). It resumes only when he says so.

## Where each piece of work was and where it stopped

| Item | Where | State |
|---|---|---|
| ROS 2 services and actions (one file per action), vision and GPS modes, prearm checks, out-and-back mission | this branch | done, pushed |
| Keep the compass present in vision mode (`EKF2_MAG_TYPE` 5, `SYS_HAS_MAG` stays 1) | `019a4a5` | done |
| Vision mode off GPS (`EKF2_GPS_CTRL` 0, navsat sensor kept), `EKF2_EV_DELAY` 0 | `b65ee37` | done |
| Five control fixes from Flight Mechanics, plus the next tier (hold re-seed, zero `cmd_vel`, GoTo yaw ENU to NED, wall clearance along the motion) | `20eed61` | done |
| Lost-tracking detection from RTAB-Map (null pose, covariance 9999, `odom_info` lost; 0.3 s age check on sim time) | `20eed61` | done |
| Prearm waits for vision yaw instead of reporting a missing compass | `5c3e60f` | done |
| Baro height (`EKF2_EV_CTRL` 9, `EKF2_HGT_REF` 0), `EKF2_RNG_CTRL` 1, `GF_MAX_VER_DIST` 3 m, `GF_ACTION` 5 (Land in v1.17; 3 is Return), e2e height-error abort, "too good to be true" grading | `8a5cf5f` | done |
| Lidar-vs-EKF2 height guard (0.3 m for 0.5 s, 0.19 m mount offset, tilt-compensated, armed above 0.5 m with valid samples), `CameraType` stereo/rgbd option | `44aa005`, `7e7887f` | done |
| Crash report for the last vision flight (lost tracking on the forward leg, climbed to about 6.5 m, crashed; ground-truth leak through the gz bridge made the hover invalid) | cloud agent | stopped, final report not delivered |
| Collision check on that crash | cloud agent queue | not started |
| Wall positions from #21's `world` to `spawn` transform, accept `"frame": "world_enu"` / `"local"`, world-to-local transform worked out at runtime once yaw is aligned and frozen at arming, 90 deg spawn test in both modes | cloud agent queue | not started |
| Brake-hold then land on vision loss during any action, takeoff included | cloud agent queue | not confirmed in code yet |
| Cleanup and optimization: split `node.py`, per-action modules, loop benchmark, 50 Hz vs 20 Hz, one switch file, golden snapshot tests, sim-time `dt`, startup check against PX4 limits, Python 3.12 | cloud agent queue | not started |
| Docstrings, `px4_control/README.md`, `includes/gz/params/README.md` | cloud agent queue | not started |
| Vision out-and-back on UFO, `cpu` profile, CPU only, separate worktree | UFO (4th in queue) | not started |

## Order when work resumes

1. Check this branch and fetch the crash report, then run the collision check.
2. Wall transform from #21's TF, with the 90 deg spawn tests.
3. Brake-hold then land on vision loss during any action.
4. Cleanup and optimization of the mission code, then comments and READMEs.
5. After #19 merges, rebase: EKF2 settings into `ekf2_vision.params` via `PX4_PARAM_FILES`, `docker compose exec` instead of `docker exec px4_sim`, drop the post-boot `param set`, check init-only params apply before ekf2 starts, braking and speed below `MPC_ACC_HOR` / `MPC_XY_VEL_MAX`, QGC `.params` converter, and document that a real Pixhawk needs the `distance_sensor` line in `dds_topics.yaml`. #21 merges before #22.
6. UFO flight, then post numbers for the Tester and take the PR out of draft.

Parked: outdoor mode (compass yaw plus vision position) on its own branch. Later: observability and failure handling.
