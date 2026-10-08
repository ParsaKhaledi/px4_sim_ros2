# scripts

Launch-time patches and offline helpers for the Gazebo tree. `gz_start_px4_gz_sim.sh` runs the origin script and the x500 patches before SITL. None of these need a running simulator for their unit tests.

## Contents

| Script | Role |
|--------|------|
| `sim_origin.py` | World origin from the spawn geographic coordinate |
| `patch_x500_ground_truth.py` | Ground-truth odometry plugin on x500 models |
| `patch_x500_lidar_down.py` | Single-ray downward lidar on x500_depth |
| `patch_x500_sensor_systems.py` | IMU, air pressure, magnetometer, and NavSat once on x500_base |
| `wall_geometry.py` | Write [walls/](../walls/README.md) |
| `fuel_assets.py` | Download Fuel models and rewrite world URIs |
| `check_offline_worlds.py` | Fail if a world still names a Fuel host |

## Run

```bash
python3 includes/gz/scripts/sim_origin.py --dry-run
python3 includes/gz/scripts/check_offline_worlds.py
python3 includes/gz/scripts/wall_geometry.py
python3 includes/gz/scripts/fuel_assets.py
```

The patch scripts edit PX4's model files when those paths exist (`~/PX4-Autopilot` or `/home/px4/PX4-Autopilot`). On a machine without that tree they print a warning and exit 0, except the sensor-system script, which exits 1 when the launched world file is missing.

## Test

```bash
python3 -m pytest includes/gz/scripts
```

## Environment

| Variable | Default | Used by |
|----------|---------|---------|
| `SIM_ORIGIN_LAT` | `35.7048378` | `sim_origin.py`. Spawn latitude, degrees |
| `SIM_ORIGIN_LON` | `51.4095049` | Spawn longitude |
| `SIM_ORIGIN_ALT` | `1205` | Spawn ground altitude, metres AMSL |
| `PX4_GZ_MODEL_POSE` | `-3,-1.6,0.15,0,0,3.14` | Spawn offset from the world origin. z is drop clearance and does not change altitude |
| `LIDAR_DOWN` | `1` | `0` removes the downward lidar block |
| `LIDAR_DOWN_RATE_HZ` | `30` | Lidar `update_rate` |
| `PX4_GZ_MODELS` | PX4 tree `Tools/simulation/gz/models` | Model lookup |
| `PX4_GZ_WORLDS` | PX4 tree `Tools/simulation/gz/worlds` | World lookup |
| `GZ_SIM_SERVER_CONFIG_PATH`, `PX4_GZ_SERVER_CONFIG` | PX4 `gz_bridge/server.config` | Sensor-system strip |

`sim_origin.py` prints `export PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT` for the shell to `eval`. In PX4 v1.17 those three are the world origin, not the vehicle home. The world origin is the spawn shifted by the pose x and y only. Heading stays 0, so world +X is east and +Y is north.

## Topics

The ground-truth plugin publishes gz `/ground_truth/odom` at 50 Hz and the pose with covariance on `/ground_truth/odom_with_covariance`. The covariance topic is set on purpose. The plugin default is `/model/<model>/odometry_with_covariance`, and PX4's GZBridge feeds that topic to EKF2 as external vision.

The lidar names (`lidar_sensor_link`, sensor `lidar`) are what GZBridge subscribes to. The patch does not publish a ROS topic itself.

## Outputs

* `sim_origin.py` rewrites `<spherical_coordinates>` on the copies PX4 loads, including `default.sdf`. The worlds committed in this repo are left as they are.
* The patch scripts rewrite `model.sdf` in the PX4 tree. A second run is a no-op when the block is already correct.
* `wall_geometry.py` writes `includes/gz/walls/<world>.json`. The schema is in the [walls README](../walls/README.md).
* `fuel_assets.py` fills `includes/gz/models/` and rewrites Fuel URIs to `model://`.
