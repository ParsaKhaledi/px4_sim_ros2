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

`patch_x500_ground_truth.py` adds a Gazebo `OdometryPublisher` to `x500_depth` and to `PX4_GZ_MODEL` (headless CI uses `x500`) before SITL starts. The plugin publishes the true model pose at 50 Hz on gz topic `/ground_truth/odom`, and the pose with covariance on `/ground_truth/odom_with_covariance`. That covariance topic is set in the plugin. The default is `/model/<model_name>/odometry_with_covariance`, and PX4 v1.17 GZBridge subscribes to exactly that topic and feeds it to EKF2 as external vision, so the default would inject the true pose into the estimator. `config_gz_bridge_sim.yaml` bridges `/ground_truth/odom` to `nav_msgs/Odometry`. Preflight runs `gz topic -i` on `/model/<model>/odometry_with_covariance` (model name from `PX4_GZ_MODEL` or `PX4_SIM_MODEL` plus the instance, default `x500_depth_0`) and fails when that topic has a publisher. It reports SKIP when the `gz` CLI is absent.

* `header.frame_id`: `world` (ENU)
* `child_frame_id`: `base_link_gt` (`GT_CHILD_FRAME`)
* stamp: Gazebo sim time

`PX4_GZ_MODEL_POSE` (default `-3,-1.6,0.15,0,0,3.14`, x,y,z,roll,pitch,yaw in radians) is the spawn pose. `sim_monitor.spawn_frame` publishes static TF `world` -> `spawn` from that x, y, and yaw with z = 0, so the frame sits on the ground under the drone and takeoff heights relative to `spawn` are heights above the floor. It also publishes TF `world` -> `base_link_gt` from `/ground_truth/odom`. The child frame is not `base_link`. RTAB-Map publishes `odom` -> `base_link`, and a second parent splits the TF tree. `trajectory_eval` scores the pose inside the odometry message, so it does not depend on that child frame name.

If the variable is unset, `gz_start_px4_gz_sim.sh` fills in the default. Compose forwards `PX4_GZ_MODEL_POSE` when `.env` sets it, which matches the devops change that moves the pose into `.env`.

## Geographic origin

`SIM_ORIGIN_LAT`, `SIM_ORIGIN_LON` and `SIM_ORIGIN_ALT` are the spawn pose, in degrees and metres AMSL. The defaults are the Aerospace Engineering Department, Amirkabir University of Technology, Tehran: 35.7048378 N, 51.4095049 E, 1205 m (SRTM 30 m gives 1206, Open-Meteo 1204).

The spawn is not the Gazebo world origin. With the default pose `-3,-1.6,0.15` and `heading_deg` 0, world +X is east and world +Y is north, so the world origin sits 3 m east and 1.6 m north of the spawn. Pose z is a drop clearance above the floor, so the world-origin altitude stays `SIM_ORIGIN_ALT` and a z of 0.15 does not change `PX4_HOME_ALT`. `includes/gz/scripts/sim_origin.py` computes that origin at launch and rewrites `<spherical_coordinates>` (EARTH_WGS84, ENU, heading 0) in the worlds PX4 loads, including every world copied from `includes/gz/worlds` and `default.sdf`. The committed SDF files keep their previous coordinates so they do not drift; the rewrite happens on the copies in the PX4 tree.

PX4 v1.17 `px4-rc.gzsim` does the same job a second time. If `PX4_HOME_LAT`, `PX4_HOME_LON` and `PX4_HOME_ALT` are all set, it calls `/world/<name>/set_spherical_coordinates` with those values as the **world origin**, before the model is spawned. The start script exports the computed world origin through those three variables so the service matches the SDF. They are not the vehicle home. The vehicle home is the first GPS fix.

GPS comes from the NavSat sensor on `x500_base` `base_link`. That link is 0.24 m above the model origin, and the sensor has no further position offset, so latitude and longitude match the spawn while the reported AMSL is `SIM_ORIGIN_ALT + spawn_z + 0.24`. With the default pose that is 1205.39 m. There is no `sensor_gps_sim` path in the gz init script; the gz NavSat reading is the one PX4 uses.

## Offline models

`apt_world` uses the local apartment mesh (`model://apt`). The committed worlds do not contact Fuel at runtime.

```bash
python3 includes/gz/scripts/check_offline_worlds.py
```

The check fails if any world under `includes/gz/worlds/` still contains `fuel.gazebosim.org` or `fuel.ignitionrobotics.org`.

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

`record` also writes `spawn.json` in the output directory. The fields are `spawn_xyz`, `spawn_yaw` (ENU, from the live `world` -> `spawn` transform), `px4_offset_s` (the lowest `t_ros - t_px4` gap between `/clock` and `/fmu/out/vehicle_odometry` over at least 50 messages), `px4_offset_spread_s`, `px4_offset_samples`, and `px4_offset_reason`.

Unit tests cover the frame conversions, the geodetic conversion, alignment, and the metrics:

```bash
python3 -m pytest
```

## Downward lidar

`LIDAR_DOWN=1` (the default) makes `patch_x500_lidar_down.py` add a single-ray downward lidar to `x500_depth` before SITL starts. `LIDAR_DOWN=0` leaves that sensor off. The block matches PX4 v1.17 `x500_lidar_down`:

* `model://LW20` included at pose `0 0 -0.079 0 1.57 0` relative to `base_link`
* fixed joint `lidar_model_joint` from `base_link` to `lw20_link`
* fixed joint `lidar_sensor_joint` from `base_link` to `lidar_sensor_link`
* link `lidar_sensor_link` at pose `0 0 -0.05 0 1.57 0` relative to `base_link`, mass `0.001`
* sensor `lidar`, type `gpu_lidar`, `gz_frame_id` `lidar_sensor_link`, sensor pose `0 0 0 3.14 0 0`
* one horizontal sample and one vertical sample (minimum and maximum angle 0), range 0.1 m to 100 m, resolution 0.01, `always_on` 1, `visualize` false

`LIDAR_DOWN_RATE_HZ` (default 30) is the sensor `update_rate`. The world file is not changed. The rendering `Sensors` system already in the world produces the `gpu_lidar` scan. PX4's GZBridge subscribes to `/world/<world>/model/<model>/link/lidar_sensor_link/sensor/lidar/scan` and sets the distance sensor downward when that sensor's world orientation is quaternion (0, 1, 0, 0). The link name and the sensor name are what make that subscription match.

## Preflight

`sim_monitor.preflight_check` serves `/sim/preflight_check` (`std_srvs/Trigger`). `success` is true when every check passed. `message` is the reason list, one check per line, each starting with `PASS`, `FAIL`, or `SKIP`. A `SKIP` line means that check did not run and does not fail the service.

Checks:

* `/sim/real_time_factor` >= `PREFLIGHT_MIN_RTF` and `/clock` is fresh and advancing. When `PREFLIGHT_MIN_RTF` is unset, the minimum is 0.15 if `HEADLESS_SOFTWARE` is 1 (measured about 0.20 on `apt_world` and 0.47 on `default.sdf` with llvmpipe) and 0.8 otherwise. A failed real-time-factor line adds `; on CPU-only machines set VISION_PROFILE=cpu (measured ~0.45 vs 0.125 for full on apt_world)` when `VISION_PROFILE` is unset or not `cpu`.
* camera, IMU, and `/ground_truth/odom` rates in sim time, over a window that ends at the current `/clock` time. A topic whose newest sample is older than three of its periods (and at least 0.5 s of sim time) fails with `stale: last message X s sim ago`. The camera rate is read from the matching `camera_info` topic (`.../image_raw` becomes `.../camera_info`, or `PREFLIGHT_CAMERA_INFO_TOPIC`), so the raw image is not subscribed. The line also shows the wall rate, for example `/ground_truth/odom: 50.0 Hz sim (10.4 Hz wall)`. Messages without a header fall back to wall Hz divided by the measured real-time factor. Camera minimums are half of `CAM_RATE_HZ` or the `VISION_PROFILE` camera rate (`full` 30 Hz, `cpu` 10 Hz). The IMU minimum is `PREFLIGHT_MIN_IMU_HZ` when set. Otherwise the expected rate is `IMU_RATE_HZ`, else `geometry.profile_from_env().imu_hz` when that module imports, else the Oak-D model's IMU `update_rate` (50 Hz here), else 50 Hz for oak or 80 Hz for px4. The minimum is 0.75 of that rate for every source. A full profile of 200 Hz fails a 100 Hz sim-time IMU. A cpu profile of 100 Hz passes 95 Hz and fails 70 Hz. The oak model rate of 50 Hz has a minimum of 37.5 Hz, and the px4 fallback of 80 Hz has a minimum of 60 Hz. The minimum is never a fixed 50 Hz. Rates are sim time. XRCE copies `sensor_combined` at most once per 10 ms, so 100 Hz is a ceiling. `PREFLIGHT_MIN_CAMERA_HZ` overrides the camera minimum. Ground truth stays at `PREFLIGHT_MIN_GT_HZ` (default 20). The IMU topic is `/imu`. The check also requires a TF path from that message's `frame_id` to the camera optical frame (`camera_rgb_optical_frame`, or `stereo_left_camera_optical_frame` when `CameraType=stereo`).
* RTAB-Map `/rtabmap/odom_info` has `lost=false`, and the RTAB-Map pose relative to its start is within `PREFLIGHT_MAX_POSE_ERR_M` (default 0.10 m) of ground truth relative to its start
* PX4 `pre_flight_checks_pass` on `vehicle_status` or `vehicle_status_vN` (highest version)
* EKF2 external-vision fusion: any of `cs_ev_pos`, `cs_ev_vel`, `cs_ev_hgt`, `cs_ev_yaw` on `estimator_status_flags` or a versioned name
* When `LIDAR_DOWN=1`, downward distance: read `/fmu/out/distance_sensor` (or `distance_sensor_vN`) and pass when `current_distance` is within 0.1 m to 0.5 m. PX4 v1.17 `dds_topics.yaml` does not publish `/fmu/out/distance_sensor` (the file only subscribes to `/fmu/in/distance_sensor`), so the check reports SKIP until that topic is in the ROS graph. `LIDAR_DOWN=0` omits the check.
* TF pairs in `PREFLIGHT_TF_PAIRS` (default `world:spawn`, `world:base_link_gt`, `base_link:imu_link`, and `base_link:camera_rgb_frame`, or `base_link:stereo_left_camera_frame` when `CameraType=stereo`)

Thresholds are environment variables, listed in `.env.example` and forwarded by Compose. The bridge script starts the service next to the real-time-factor publisher and the spawn frame.

```bash
ros2 service call /sim/preflight_check std_srvs/srv/Trigger
```

`IMU_SOURCE=oak` bridges gz `/imu` to ROS `/imu` from `config_gz_bridge_imu.yaml`. That camera IMU (`frame_id` `imu_link`) is the only `/imu` publisher. RTAB-Map reads it and publishes `/rtabmap/odom` with `frame_id` `base_link`. The camera frames in `x500_urdf.urdf` are `camera_link`, `imu_link`, `camera_rgb_frame`, `stereo_left_camera_frame`, and the optical frames. Camera image topics stay in `config_gz_bridge.yaml`. Do not add a second `/imu` bridge there.

`patch_x500_sensor_systems.py` adds `gz::sim::systems::Imu`, `AirPressure`, and `NavSat` to PX4's `x500_base` model once, and does not change sensor elements. Startup strips those three from the world file Gazebo loads and from PX4's `server.config`. The magnetometer is a world plugin, not a `sim.params` line. `apt_world` includes `gz::sim::systems::Magnetometer` so PX4 sees a compass, and startup removes that plugin from `server.config` when the world already has it. `default` has no magnetometer plugin of its own, so it keeps the one in `server.config`. Spawning a second x500 in the same world would load the model systems twice; multi-vehicle would need them back in one shared place, which is out of scope.

The start scripts resolve `includes/gz` with `SIM_GZ_DIR`, then `SCRIPT_DIR/..` when `scripts/sim_origin.py` is there, then `/home/px4/volume/includes/gz` (the compose mount; `startFiles` is mounted separately at `/home/px4/volume/startFiles`). A world name other than `default` is passed as `PX4_GZ_WORLD` to `make px4_sitl gz_<model>`. `gz_<model>_<world>` is not a ninja target for these worlds.
