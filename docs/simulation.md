# Simulation

Gazebo Harmonic (gz) runs inside the PX4 container. Worlds and models under `includes/gz/` are copied into the PX4 tree at container start, so these files do not need an image rebuild.

## Frames

| Name | Axes | Used for |
|------|------|----------|
| ENU world | x east, y north, z up | Gazebo world, `/ground_truth/odom`, spawn TF, wall maps |
| FLU body | x forward, y left, z up | ROS `base_link`, RTAB-Map odometry |
| NED world | x north, y east, z down | PX4 local position (`vehicle_odometry.position`) |
| FRD body | x forward, y right, z down | PX4 attitude (`vehicle_odometry.q`, body to NED) |

`/ground_truth/odom` is the Gazebo model pose in ENU. PX4 `vehicle_odometry` is converted to ENU/FLU before it is compared with ground truth: position `(east, north, up) = (ned_y, ned_x, -ned_z)`, and the quaternion is the px4_ros_com NED/FRD to ENU/FLU rotation. GPS latitude/longitude/altitude is converted to metres in an ENU frame whose origin is the first fix (the home sample in that recording). It is not locked to `SIM_ORIGIN_*`, so a bag that starts away from the spawn still lines up with itself.

The static TF `world` -> `spawn` is the spawn pose, so a pose can be expressed relative to the takeoff point by transforming through `spawn`.

## Headless Gazebo

`HEADLESS=0` (the default) starts the gz server and the gz GUI. The GUI needs `DISPLAY` and `xhost`.

`HEADLESS=1` starts only the server. PX4 already passes `-s` and skips `gz sim -g`. A small `gz` wrapper adds `--headless-rendering` so camera sensors still render through EGL, with no X server. Set `HEADLESS_BACKEND=xvfb` to use a virtual X display instead (the image must contain `Xvfb`). Set `HEADLESS_SOFTWARE=1` to force Mesa software OpenGL on a machine with no GPU.

```bash
HEADLESS=1 CameraType=rgbd World=default ./scripts/up.sh
```

`HEADLESS`, `HEADLESS_BACKEND`, and `HEADLESS_SOFTWARE` are in `.env.example` and are passed into the PX4 service by `docker-compose-px4.yml`.

## Sim time

Nav2 (`navigation_launch.py` and `start_nav2.sh`), Nav2 RViz, the x500 `robot_state_publisher`, and the ros_gz bridge use sim time. The bridge publishes `/clock` from Gazebo. Override with `USE_SIM_TIME=false`. RTAB-Map launch scripts are owned by the vision work and are not changed here.

## Real-time factor

`sim_monitor.real_time_factor` reads `/world/<name>/stats` and publishes `std_msgs/Float64` on `/sim/real_time_factor`. The value is Δsim_time/Δwall_time over `SIM_RTF_WINDOW_S` (default 5 s), not Gazebo's own `real_time_factor` field. That field read about 0.03 on a software-rendered CPU run while sim time advanced at about 0.21 of wall time. The raw field is on `/sim/real_time_factor_gz`. The node logs about once every five seconds. The bridge script starts it.

## Ground truth and the spawn frame

`patch_x500_ground_truth.py` adds a Gazebo `OdometryPublisher` to `x500_depth` before SITL starts. The plugin publishes the true model pose at 50 Hz on gz topic `/ground_truth/odom`. `config_gz_bridge_sim.yaml` bridges it to `nav_msgs/Odometry`.

* `header.frame_id`: `world` (ENU)
* `child_frame_id`: `base_link_gt` (`GT_CHILD_FRAME`)
* stamp: Gazebo sim time

`PX4_GZ_MODEL_POSE` (default `-3,-1.6,0,0,0,3.14`, x,y,z,roll,pitch,yaw in radians) is the spawn pose. `sim_monitor.spawn_frame` publishes static TF `world` -> `spawn` from that value, and TF `world` -> `base_link_gt` from `/ground_truth/odom`. The child frame is not `base_link`. RTAB-Map publishes `odom` -> `base_link`, and a second parent splits the TF tree. `trajectory_eval` scores the pose inside the odometry message, so it does not depend on that child frame name.

If the variable is unset, `gz_start_px4_gz_sim.sh` fills in the default. Compose forwards `PX4_GZ_MODEL_POSE` when `.env` sets it, which matches the devops change that moves the pose into `.env`.

## Geographic origin

`SIM_ORIGIN_LAT`, `SIM_ORIGIN_LON` and `SIM_ORIGIN_ALT` are the spawn pose, in degrees and metres AMSL. The defaults are the Aerospace Engineering Department, Amirkabir University of Technology, Tehran: 35.7048378 N, 51.4095049 E, 1205 m (SRTM 30 m gives 1206, Open-Meteo 1204).

The spawn is not the Gazebo world origin. With the default pose `-3,-1.6,0` and `heading_deg` 0, world +X is east and world +Y is north, so the world origin sits 3 m east and 1.6 m north of the spawn, at the same elevation when spawn z is 0. `includes/gz/scripts/sim_origin.py` computes that origin at launch and rewrites `<spherical_coordinates>` (EARTH_WGS84, ENU, heading 0) in the worlds PX4 loads, including every world copied from `includes/gz/worlds` and `default.sdf`. The committed SDF files keep their previous coordinates so they do not drift; the rewrite happens on the copies in the PX4 tree.

PX4 v1.17 `px4-rc.gzsim` does the same job a second time. If `PX4_HOME_LAT`, `PX4_HOME_LON` and `PX4_HOME_ALT` are all set, it calls `/world/<name>/set_spherical_coordinates` with those values as the **world origin**, before the model is spawned. The start script exports the computed world origin through those three variables so the service matches the SDF. They are not the vehicle home. The vehicle home is the first GPS fix.

GPS comes from the NavSat sensor on `x500_base` `base_link`. That link is 0.24 m above the model origin, and the sensor has no further position offset, so latitude and longitude match the spawn while the reported AMSL is `SIM_ORIGIN_ALT + spawn_z + 0.24`. With the default pose that is 1205.24 m. There is no `sensor_gps_sim` path in the gz init script; the gz NavSat reading is the one PX4 uses.

## Offline models

`husarion_office.sdf` referenced Fuel meshes, and `sonoma_raceway.sdf` included the Sonoma Raceway model from Fuel. `includes/gz/scripts/fuel_assets.py` downloads those files into `includes/gz/models/` and rewrites the URIs to `model://`. The committed worlds do not contact Fuel at runtime.

```bash
python3 includes/gz/scripts/check_offline_worlds.py
python3 includes/gz/scripts/fuel_assets.py   # re-download if a world gains a Fuel URI
```

The check fails if any world under `includes/gz/worlds/` still contains `fuel.gazebosim.org` or `fuel.ignitionrobotics.org`.

Fuel furniture models keep their upstream licence (see each `model.config` when Fuel provided one). The Sonoma Raceway model is CC0.

## Wall maps

`includes/gz/scripts/wall_geometry.py` slices each world's collision geometry with a horizontal plane at `z = 0.5 m` and writes `includes/gz/walls/<world>.json`.

```json
{
  "world": "apt_world",
  "frame": "world_enu",
  "units": "m",
  "slice_z_m": 0.5,
  "segment_count": 0,
  "segments": [
    {"start": [0.0, 0.0], "end": [1.0, 0.0], "source": "model/link/collision", "kind": "box"}
  ]
}
```

`start` and `end` are `[x, y]` in the Gazebo ENU world, in metres. `kind` is `box`, `cylinder`, or `mesh`. Shapes shorter than 0.25 m (floors) are skipped. Segments shorter than 5 cm are dropped. `unresolved_uris` lists meshes that were not on disk. A control node can compute the distance from a point to the nearest segment.

Regenerate with:

```bash
python3 includes/gz/scripts/wall_geometry.py
```

## trajectory_eval

Package: `includes/gz/sim_ws/src/trajectory_eval`. It does not need `colcon` for the math; the container can run it with `PYTHONPATH` pointed at the package.

Recorded streams, all in ENU metres:

| Name | Source |
|------|--------|
| `ground_truth` | `/ground_truth/odom` |
| `gps` | `/fmu/out/vehicle_gps_position` (or `vehicle_gps_position_vN`), WGS84 to ENU about the first fix in the recording. At a cold start that fix is the spawn (`SIM_ORIGIN_*`, plus the 0.24 m NavSat height above the model origin). |
| `rtabmap` | `/rtabmap/odom` |
| `ekf2` | `/fmu/out/vehicle_odometry` (or a versioned name), NED/FRD to ENU/FLU |

Topics are flags (`--gt-topic`, `--gps-topic`, `--rtabmap-topic`, `--ekf-topic`). Live recording uses the node clock, so `use_sim_time` puts every stamp on the sim timeline. A rosbag is read with its recorded timestamps.

```bash
export PYTHONPATH=includes/gz/sim_ws/src/trajectory_eval
python3 -m trajectory_eval record --duration 20 --output /tmp/traj
python3 -m trajectory_eval bag --bag /path/to/bag --output /tmp/traj
python3 -m trajectory_eval offline --ground-truth gt.tum --rtabmap rtab.tum --ekf ekf.tum --output /tmp/traj
```

Each stream is written as a TUM file (`timestamp tx ty tz qx qy qz qw`) for evo. Estimates are resampled onto ground-truth stamps before scoring. `metrics.json` and `metrics.csv` contain:

* absolute trajectory error RMS, mean, and max, unaligned relative to takeoff (each trajectory minus its first position) and after an SE(3) alignment with scale fixed at 1
* relative pose error on ground-truth segments of 1 m and 5 m, as metres, metres per metre, and percent of distance (null when the path is shorter than the segment)
* final position error, both ways
* height error and yaw error (absolute, and with the initial yaw removed). GPS yaw is omitted because a fix has no body heading.
* GPS versus ground truth: residual RMS and per-axis mean and standard deviation

`trajectories_xy.png` and `ate_unaligned.png` are written when matplotlib is installed.

Unit tests cover the frame conversions, the geodetic conversion, alignment, and the metrics:

```bash
python3 -m pytest
```

## Preflight

`sim_monitor.preflight_check` serves `/sim/preflight_check` (`std_srvs/Trigger`). `success` is true only when every line passes. `message` is the reason list, one check per line, each starting with `PASS` or `FAIL`.

Checks:

* `/sim/real_time_factor` >= `PREFLIGHT_MIN_RTF` and `/clock` is fresh and advancing. When `PREFLIGHT_MIN_RTF` is unset, the minimum is 0.15 if `HEADLESS_SOFTWARE` is 1 (measured about 0.20 on `apt_world` and 0.47 on `default.sdf` with llvmpipe) and 0.8 otherwise.
* camera, IMU, and `/ground_truth/odom` rates in sim time. The line also shows the wall rate, for example `/ground_truth/odom: 50.0 Hz sim (10.4 Hz wall)`. Messages without a header fall back to wall Hz divided by the measured real-time factor. Camera and oak-IMU minimums are half of `CAM_RATE_HZ` / `IMU_RATE_HZ` when those are set, otherwise half of the `VISION_PROFILE` rate (`full`: camera 30 Hz, IMU 200 Hz; `cpu`: camera 10 Hz, IMU 100 Hz; no profile uses the full rates). `IMU_SOURCE=px4` uses 80 Hz in sim time. XRCE copies `sensor_combined` at most once per 10 ms, so 100 Hz is a ceiling. `PREFLIGHT_MIN_CAMERA_HZ` and `PREFLIGHT_MIN_IMU_HZ` override that. Ground truth stays at `PREFLIGHT_MIN_GT_HZ` (default 20). The IMU topic is `/imu`. The check also requires a TF path from that message's `frame_id` to the camera optical frame (`camera_rgb_optical_frame`, or `stereo_left_camera_optical_frame` when `CameraType=stereo`).
* RTAB-Map `/rtabmap/odom_info` has `lost=false`, and the RTAB-Map pose relative to its start is within `PREFLIGHT_MAX_POSE_ERR_M` (default 0.10 m) of ground truth relative to its start
* PX4 `pre_flight_checks_pass` on `vehicle_status` or `vehicle_status_vN` (highest version)
* EKF2 external-vision fusion: any of `cs_ev_pos`, `cs_ev_vel`, `cs_ev_hgt`, `cs_ev_yaw` on `estimator_status_flags` or a versioned name
* TF pairs in `PREFLIGHT_TF_PAIRS` (default `world:spawn`, `world:base_link_gt`, `base_link:imu_link`, and `base_link:camera_rgb_frame`, or `base_link:stereo_left_camera_frame` when `CameraType=stereo`)

Thresholds are environment variables, listed in `.env.example` and forwarded by Compose. The bridge script starts the service next to the real-time-factor publisher and the spawn frame.

```bash
ros2 service call /sim/preflight_check std_srvs/srv/Trigger
```

`IMU_SOURCE=oak` (the default) bridges gz `/imu` to ROS `/imu` from `config_gz_bridge_imu.yaml`. The Oak-D sensor frame is `imu_link`. `IMU_SOURCE=px4` does not bridge that topic. `px4_imu_relay` publishes `/imu` in `base_link` from `/fmu/out/sensor_combined` and `/fmu/out/vehicle_attitude`, which PX4 v1.17 exports in `dds_topics.yaml` with no version suffix. Stamps default to ROS time at receive (`IMU_STAMP_MODE=receive`). `px4_offset` uses the PX4 sample time plus one offset to `/clock`, measured once at startup. This stack does not publish `base_link` to `camera_link`, the optical frames, or `imu_link`. The vision StatePublisher does. Camera image topics stay in `config_gz_bridge.yaml`. Do not add a second `/imu` bridge there. The IMU modes are in the root README.

Each world under `includes/gz/worlds` loads NavSat, magnetometer, IMU, air pressure, and the sensors system with the ogre2 render engine. PX4's `GZ_SIM_SERVER_CONFIG_PATH` also adds those systems; the world files carry them so a world still has GPS and a compass if that server config is not applied. `apt_world` was missing the magnetometer, so `vehicle_global_position` never published and pre-arm failed.

The start scripts resolve `includes/gz` with `SIM_GZ_DIR`, then `SCRIPT_DIR/..` when `scripts/sim_origin.py` is there, then `/home/px4/volume/includes/gz` (the compose mount; `startFiles` is mounted separately at `/home/px4/volume/startFiles`). A world name other than `default` is passed as `PX4_GZ_WORLD` to `make px4_sitl gz_<model>`. `gz_<model>_<world>` is not a ninja target for these worlds.
