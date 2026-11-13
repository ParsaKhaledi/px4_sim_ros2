# trajectory_eval

Score GPS, RTAB-Map, and PX4 EKF2 against Gazebo ground truth. Poses are converted into one ENU world before they are compared. Math tests do not need a running simulator.

## Contents

| Module | Role |
|--------|------|
| `cli.py` | `record`, `bag`, and `offline` |
| `record.py` | Live subscriptions. Also writes `spawn.json` |
| `spawn.py` | The six `spawn.json` fields |
| `bag.py` | rosbag2 reader |
| `frames.py` | NED/FRD to ENU/FLU, and WGS84 to ENU |
| `metrics.py` | ATE, RPE, height, yaw |
| `report.py` | JSON, CSV, TUM, plots |
| `tum.py` | TUM files (`timestamp tx ty tz qx qy qz qw`) |

## Run

```bash
export PYTHONPATH=includes/gz/sim_ws/src/trajectory_eval
python3 -m trajectory_eval record --duration 20 --output /tmp/traj
python3 -m trajectory_eval bag --bag /path/to/bag --output /tmp/traj
python3 -m trajectory_eval offline --ground-truth gt.tum --rtabmap rtab.tum --ekf ekf.tum --output /tmp/traj
```

`record` and `bag` need ROS (`rclpy`, `px4_msgs`, and `rosbag2_py` for bags). `offline` needs numpy. Plots need matplotlib. Without it the report still writes and sets `plots_skipped`.

`--duration` is seconds of sim time. `0` waits for Ctrl-C. `--use-sim-time` defaults to true, so live stamps follow `/clock`.

## Test

```bash
python3 -m pytest includes/gz/sim_ws/src/trajectory_eval/test
```

## Environment

The tool does not read the simulation `.env`. Live recording takes:

| Flag | Default |
|------|---------|
| `--gt-topic` | `/ground_truth/odom` |
| `--gps-topic` | `/fmu/out/vehicle_gps_position` |
| `--rtabmap-topic` | `/rtabmap/odom` |
| `--ekf-topic` | `/fmu/out/vehicle_odometry` |
| `--rpe` | `1.0 5.0` (metres of ground-truth path) |
| `--use-sim-time` | true |

## Topics

| Stream | Source | Frame conversion |
|--------|--------|------------------|
| `ground_truth` | `/ground_truth/odom` | already Gazebo ENU |
| `gps` | `/fmu/out/vehicle_gps_position` or `vehicle_gps_position_vN` | WGS84 to ENU about the first fix in the recording |
| `rtabmap` | `/rtabmap/odom` | already ENU/FLU |
| `ekf2` | `/fmu/out/vehicle_odometry` or a versioned name | NED/FRD to ENU/FLU |

GPS ENU is about that recording's first fix. It is not locked to `SIM_ORIGIN_*`. PX4 position is `(east, north, up) = (ned_y, ned_x, -ned_z)`.

## Outputs

The output directory holds one TUM file per stream, `metrics.json`, and `metrics.csv`. `record` also writes `spawn.json` with `spawn_xyz`, `spawn_yaw`, `px4_offset_s`, `px4_offset_spread_s`, `px4_offset_samples`, and `px4_offset_reason`. `spawn_xyz` and `spawn_yaw` come from the live `world` -> `spawn` transform. `px4_offset_s` is the lowest `t_ros - t_px4` gap between `/clock` and `/fmu/out/vehicle_odometry` over at least 50 messages.

* absolute trajectory error, unaligned relative to each trajectory's first position, and after an SE(3) alignment with scale fixed at 1
* relative pose error on ground-truth segments of 1 m and 5 m
* final position error, height error, and yaw error. GPS yaw is omitted
* GPS residual against ground truth

`trajectories_xy.png` and `ate_unaligned.png` are written when matplotlib imports.
