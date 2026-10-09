# sim_monitor

Real-time factor, the spawn frame, and preflight. The bridge script starts these next to `ros_gz_bridge`. `/imu` is the camera bridge. The longer description is in [docs/simulation.md](../../../../../docs/simulation.md).

## Contents

| Module | Role |
|--------|------|
| `real_time_factor.py` | Δsim/Δwall from Gazebo world stats |
| `spawn_frame.py` | Static TF `world` → `spawn`, and TF from ground truth |
| `preflight_check.py` | `/sim/preflight_check` |
| `checks.py` | Rate, pose, and leak checks (no ROS node) |
| `imu_source.py` | The camera bridge is the only `/imu` publisher |
| `imu_frames.py` | FRD/NED to FLU/ENU helpers |

## Run

From a checkout, with ROS sourced:

```bash
export PYTHONPATH=includes/gz/sim_ws/src/sim_monitor
python3 -m sim_monitor.real_time_factor
python3 -m sim_monitor.spawn_frame
python3 -m sim_monitor.preflight_check
```

`includes/gz/startFiles/gz_start_sim_helpers.sh` does this inside the PX4 container. `USE_SIM_TIME` defaults to true on the spawn frame and the preflight node.

## Test

```bash
python3 -m pytest includes/gz/sim_ws/src/sim_monitor/test
```

The tests do not need a running simulator or rclpy.

## Environment

| Variable | Default | Effect |
|----------|---------|--------|
| `USE_SIM_TIME` | `true` | Node clock follows `/clock` |
| `SIM_RTF_WINDOW_S` | `5` | Window for Δsim/Δwall |
| `PX4_GZ_MODEL_POSE` | `-3,-1.6,0.15,0,0,3.14` | Spawn pose. The `spawn` frame uses x, y, yaw and z = 0 |
| `GT_CHILD_FRAME` | `base_link_gt` | Child of the ground-truth TF |
| `PREFLIGHT_GT_TOPIC` | `/ground_truth/odom` | Ground-truth odometry |
| `IMU_SOURCE` | `oak` | Bridges the camera IMU onto `/imu` |
| `IMU_STAMP_MODE` | `receive` | `px4_offset` adds one startup offset onto `/clock` |
| `PREFLIGHT_MIN_RTF` | `0.15` if `HEADLESS_SOFTWARE=1`, else `0.8` | Real-time-factor floor |
| `PREFLIGHT_MIN_GT_HZ` | `20` | Ground-truth rate floor, sim time |
| `PREFLIGHT_MIN_CAMERA_HZ` | half the camera rate | Camera rate is read from `camera_info` |
| `PREFLIGHT_MIN_IMU_HZ` | 0.75 of the expected IMU rate | Override. Unset uses `IMU_MIN_FRACTION` |
| `PREFLIGHT_CLOCK_MAX_AGE_S` | `1.0` | Wall-clock age of the last `/clock` |
| `PREFLIGHT_MAX_POSE_ERR_M` | `0.10` | RTAB-Map vs ground truth, from each stream's start |
| `PREFLIGHT_CAMERA_TOPIC` | `/camera/rgb/image_raw` | Stereo replaces the RGB default |
| `PREFLIGHT_CAMERA_INFO_TOPIC` | derived from the image topic | Camera rate subscription |
| `PREFLIGHT_IMU_TOPIC` | `/imu` | |
| `LIDAR_DOWN` | `1` | Also checks `/fmu/out/distance_sensor` |
| `CameraType` | `rgbd` | Picks the camera frame in the TF check |

`HEADLESS_SOFTWARE=1` lowers the real-time-factor floor because software GL on a CPU sim sits near 0.2. A GPU run uses 0.8.

The IMU floor is `PREFLIGHT_MIN_IMU_HZ` when set. Otherwise it is `IMU_MIN_FRACTION` (0.75) times the expected rate, for every source. That rate is `IMU_RATE_HZ`, else `geometry.profile_from_env().imu_hz` when that module imports, else the Oak-D SDF `update_rate`, else 50 Hz for oak or 80 Hz for px4. A full profile of 200 Hz fails a 100 Hz sim-time IMU (floor 150). A cpu profile of 100 Hz passes 95 Hz and fails 70 Hz (floor 75). The oak model at 50 Hz has a floor of 37.5 Hz, and the px4 fallback of 80 Hz has a floor of 60 Hz.

## Topics and services

| Name | Direction |
|------|-----------|
| `/world/<name>/stats` | gz input to the real-time-factor node |
| `/sim/real_time_factor` | `std_msgs/Float64`, Δsim/Δwall |
| `/sim/real_time_factor_gz` | Gazebo's own field, for comparison |
| `/ground_truth/odom` | input to the spawn TF and to preflight |
| `/clock` | sim time for rate gates. Freshness is wall time |
| `/sim/preflight_check` | `std_srvs/Trigger` |
| `/imu` | camera IMU, bridged from Gazebo |
| `/rtabmap/odom`, `/rtabmap/odom_info` | pose check |
| `/fmu/out/vehicle_status`, `estimator_status_flags` | PX4 checks, versioned names included |
| `/fmu/out/distance_sensor` | downward lidar, when `LIDAR_DOWN=1` |

TF out: `world` → `spawn` (static), `world` → `GT_CHILD_FRAME`.

## Outputs

Preflight `success` is true when every line passed. `message` is one check per line, each starting with `PASS`, `FAIL`, or `SKIP`. A `SKIP` line did not run. It is not a pass and it does not fail the service. The ground-truth leak check runs `gz topic -i` on `/model/<model>/odometry_with_covariance` and skips when the `gz` CLI is absent.
