# startFiles

Shell scripts the PX4 container runs. This page covers the headless server, the geographic origin, the x500 patches, sim time, and the monitor nodes. Compose mounts this directory at `/home/px4/volume/startFiles`.

## Contents

| Script | Role |
|--------|------|
| `gz_resolve_dir.sh` | Find `includes/gz` and set `GZ_DIR` |
| `gz_start_px4_gz_sim.sh` | Headless or GUI SITL, origin, and x500 patches |
| `gz_start_ros2_gz_bridge.sh` | Bridge config, then the monitor nodes |
| `gz_start_sim_helpers.sh` | Real-time factor, spawn TF, preflight, optional IMU relay |
| `start_nav2.sh` | Nav2 with sim time |
| `start_nav2_rviz.sh` | Nav2 RViz with sim time |

`includes/gz/bin/gz` is the wrapper PX4's `gz` calls go through. With `HEADLESS=1` and the EGL backend it adds `--headless-rendering` so cameras still render. GUI launches are unchanged. `REAL_GZ` is the real binary (default `/usr/bin/gz`).

## Run

The container starts these. From a shell that already has the PX4 image layout:

```bash
# argument is the world stem
./includes/gz/startFiles/gz_start_px4_gz_sim.sh default
./includes/gz/startFiles/gz_start_ros2_gz_bridge.sh
```

`gz_resolve_dir` looks at `SIM_GZ_DIR`, then the directory above the script when that tree contains `scripts/sim_origin.py`, then `/home/px4/volume/includes/gz`.

## Test

```bash
bash -n includes/gz/startFiles/*.sh
python3 -m pytest includes/gz/scripts/test_gz_resolve_dir.py
```

`bash -n` checks syntax. The resolve test checks the directory lookup without starting Gazebo.

## Environment

| Variable | Default | Effect |
|----------|---------|--------|
| `HEADLESS` | `0` | `1` runs the server with no GUI and unsets `DISPLAY` |
| `HEADLESS_BACKEND` | `egl` | `xvfb` uses a virtual X display when `Xvfb` is installed |
| `HEADLESS_SOFTWARE` | `0` | `1` sets `LIBGL_ALWAYS_SOFTWARE=1` |
| `HEADLESS_DISPLAY` | `:99` | Display for the Xvfb backend |
| `PX4_GZ_MODEL_POSE` | `-3,-1.6,0.15,0,0,3.14` | Spawn pose passed to PX4. z is drop clearance |
| `SIM_ORIGIN_LAT`, `SIM_ORIGIN_LON`, `SIM_ORIGIN_ALT` | Amirkabir, 1205 m | See [scripts/README.md](../scripts/README.md) |
| `SIM_GZ_DIR` | unset | Override for `GZ_DIR` |
| `IMU_SOURCE` | `oak` | `oak` bridges gz `/imu`. `px4` starts `px4_imu_relay` and does not bridge it |
| `USE_SIM_TIME` | `true` | Bridge, Nav2, and Nav2 RViz follow `/clock` |
| `World` argument | `default` | Non-default sets `PX4_GZ_WORLD` |

`HEADLESS=1` still renders cameras. The floor for `/sim/preflight_check` drops to 0.15 when `HEADLESS_SOFTWARE=1`.

## Topics and services

The bridge publishes `/clock` from Gazebo when `USE_SIM_TIME` is true. `config_gz_bridge_sim.yaml` adds `/ground_truth/odom`. The IMU bridge is a separate file and is appended only for `IMU_SOURCE=oak`.

`gz_start_sim_helpers.sh` then starts:

| Process | Interface |
|---------|-----------|
| `sim_monitor.real_time_factor` | publishes `/sim/real_time_factor` |
| `sim_monitor.spawn_frame` | TF `world` → `spawn` and `world` → `base_link_gt` |
| `sim_monitor.preflight_check` | serves `/sim/preflight_check` |
| `sim_monitor.px4_imu_relay` | publishes `/imu` when `IMU_SOURCE=px4` |

Rate checks use sim time. The discovery timers in those nodes use a steady wall clock so they keep running when `/clock` is slow.

## Outputs

`gz_start_px4_gz_sim.sh` evals the `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT` lines from `sim_origin.py`. Those are the world origin. The script then patches x500 models in the PX4 tree and execs `make px4_sitl gz_x500_depth`. A missing launched world stops the script before SITL. A missing ground-truth or lidar model file warns and continues.
