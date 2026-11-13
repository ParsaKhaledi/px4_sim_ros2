# scripts

Launch-time patches and offline helpers for the Gazebo tree. `gz_start_px4_gz_sim.sh` runs the origin script and the x500 patches before SITL. None of these need a running simulator for their unit tests.

## Contents

| Script | Role |
|--------|------|
| `sim_origin.py` | World origin from the spawn geographic coordinate |
| `patch_x500_ground_truth.py` | SITL start: patch spawned `x500` / `x500_depth` in PX4; `--repo` refreshes baked overlay |
| `patch_x500_lidar_down.py` | Single-ray downward lidar on x500_depth |
| `patch_x500_sensor_systems.py` | IMU, air pressure, and NavSat once on x500_base. Magnetometer stays a world plugin |
| `fuel_assets.py` | Download Fuel models and rewrite world URIs |
| `check_offline_worlds.py` | Fail if a world still names a Fuel host |

## Run

```bash
python3 includes/gz/scripts/sim_origin.py --dry-run
python3 includes/gz/scripts/check_offline_worlds.py
python3 includes/gz/scripts/fuel_assets.py
```

`gz_start_px4_gz_sim.sh` runs `patch_x500_ground_truth.py` before SITL so flight CI's plain `x500` (and `x500_depth`) get `/ground_truth/odom`. Without that patch the ros_gz bridge still advertises the topic and the health check fails at 0 Hz. `x500_depth` is also baked under `includes/gz/models/x500_depth`; refresh that overlay with `--repo` after stock layout changes. `gz_modifications.bash` copies the overlay into the PX4 model tree at container start.

The lidar and sensor-system patch scripts still edit PX4's model files when those paths exist (`~/PX4-Autopilot` or `/home/px4/PX4-Autopilot`). On a machine without that tree they print a warning and exit 0, except the sensor-system script, which exits 1 when the launched world file is missing.

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

The ground-truth plugin is baked into `includes/gz/models/x500_depth/model.sdf` and is also applied at SITL start to the spawned airframe (CI uses `x500`). It publishes gz `/ground_truth/odom` at 50 Hz and the pose with covariance on `/ground_truth/odom_with_covariance`. The covariance topic is set on purpose. The plugin default is `/model/<model>/odometry_with_covariance`, and PX4's GZBridge feeds that topic to EKF2 as external vision.

```bash
python3 includes/gz/scripts/patch_x500_ground_truth.py                 # SITL: patch PX4 x500 models
python3 includes/gz/scripts/patch_x500_ground_truth.py --repo          # refresh baked x500_depth
python3 includes/gz/scripts/patch_x500_ground_truth.py --repo --reset-stock
```

The lidar names (`lidar_sensor_link`, sensor `lidar`) are what GZBridge subscribes to. The patch does not publish a ROS topic itself.

## Outputs

* `sim_origin.py` rewrites `<spherical_coordinates>` on the copies PX4 loads, including `default.sdf`. The worlds committed in this repo are left as they are.
* `patch_x500_ground_truth.py` (default) rewrites PX4 `x500*` `model.sdf` files that exist. `--repo` rewrites only the baked repo `models/x500_depth/model.sdf`. A second run is a no-op when the plugin is already correct.
* The lidar and sensor-system patch scripts rewrite `model.sdf` in the PX4 tree. A second run is a no-op when the block is already correct.
* `fuel_assets.py` fills `includes/gz/models/` and rewrites Fuel URIs to `model://`.
