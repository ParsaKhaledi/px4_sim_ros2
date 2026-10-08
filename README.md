# PX4 Simulation with ROS 2

Docker-orchestrated PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy simulation stack. Pull a pre-built image from [Docker Hub](https://hub.docker.com/r/alienkh/px4_sim) and launch with Docker Compose.

## Version Matrix

| Component | Version |
|-----------|---------|
| Stack release | v3.0.0 |
| PX4-Autopilot | v1.17.0 |
| px4_msgs | [v1.17.0](https://github.com/PX4/px4_msgs/releases/tag/v1.17.0) (tag `v${PX4_VERSION}`) |
| px4_ros_com | main (examples only) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (`gz-harmonic`) |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Micro-XRCE-DDS-Agent | v2.4.3 |

## Quick Start

```bash
# Allow Docker containers to use the host display (for QGC / RViz)
xhost +local:

# Configure image tag from Docker Hub or a local build
cp .env.example .env

# Start full stack (core + GCS + SLAM + navigation)
CameraType=rgbd World=default ./scripts/up.sh

# Or start only the simulation core
COMPOSE_PROFILES= ./scripts/up.sh
```

### Environment variables

| Variable | Description | Default |
|----------|-------------|---------|
| `registry` | Image registry | `docker.io/alienkh` |
| `px4TAG` | Image tag in `.env` | `v3.0.0` |
| `CameraType` | `rgbd` or `stereo` | `rgbd` |
| `World` | Gazebo world filename stem | `default` |
| `COMPOSE_PROFILES` | Comma-separated profiles | `gcs,slam,nav` |
| `CAM_PITCH_DEG` | Downward camera pitch in degrees (positive looks down) | `17` |
| `CAM_X`, `CAM_Y`, `CAM_Z` | Camera link origin on `base_link`, metres | `0.12`, `0.03`, `0.242` |
| `RTABMAPVIZ` | Open the RTAB-Map viewer | `false` |
| `SLAM_APE_RMS_MAX` | Pass/fail APE RMSE for `scripts/eval_slam_accuracy.py`, metres | `0.50` |
| `SLAM_DRIFT_PER_M_MAX` | Pass/fail drift per metre (RPE over 1 m) | `0.10` |

CI publishes tags like `v3.0.0` and `v3.0.0-latest` (GPU: `v3.0.0_GPU`, `v3.0.0-latest_GPU`). Update `px4TAG` in `.env` after pulling a new build.

### Compose profiles

| Profile | Services |
|---------|----------|
| *(none)* | PX4, StatePublisher |
| `gcs` | QGroundControl |
| `slam` | RTAB-Map |
| `nav` | Nav2 + RViz (use with `slam`) |

```bash
# Simulation + ground station only
COMPOSE_PROFILES=gcs CameraType=rgbd World=apt_world ./scripts/up.sh

# Full robotics stack
COMPOSE_PROFILES=gcs,slam,nav CameraType=rgbd World=default ./scripts/up.sh

# Stereo camera in a custom world
COMPOSE_PROFILES=gcs CameraType=stereo World=husarion_office ./scripts/up.sh
```

You can also set `CameraType` and `World` in `.env` instead of the command line.

### Valid worlds

World names match SDF **filenames** under `includes/gz/worlds/` (without `.sdf`):

- `default` — PX4 default world
- `apt_world`
- `husarion_office`
- `husarion_world`
- `sonoma_raceway`
- `empty_with_plugins`

### Runtime configuration (brief)

**How `CameraType` and `World` are applied:** `scripts/up.sh` exports both variables into Compose. The PX4 container runs `gz_modifications.bash` (camera + custom models/worlds) then `gz_start_px4_gz_sim.sh` (world/make target). If the `slam` profile is active, Rtabmap receives the same `CameraType`.

**Changing world or camera on a running stack:** These are applied at container start. Bring the stack down first, or recreate PX4:

```bash
docker compose -f docker-compose-px4.yml down
CameraType=stereo World=apt_world ./scripts/up.sh
# or: CameraType=stereo World=apt_world docker compose -f docker-compose-px4.yml up -d --force-recreate PX4
```

**Verify:** `docker logs px4_sim 2>&1 | grep "Selected Camera Type"`

**Profile dependencies:** `nav` depends on `slam` (Nav2 waits for Rtabmap). Use `COMPOSE_PROFILES=slam,nav` or include both. Services without a profile (`PX4`, `StatePublisher`) always start.

**Without `up.sh`:** `CameraType=rgbd World=default docker compose -f docker-compose-px4.yml --profile gcs up -d`

**Display / X11:** QGC and RViz need `DISPLAY` and `xhost +local:`. Set `XAUTH` if your setup uses `/tmp/.docker.xauth` (Compose may warn if unset).

| Service | Container | Role |
|---------|-----------|------|
| PX4 | `px4_sim` | SITL, Gazebo, XRCE agent, ros_gz bridge |
| StatePublisher | `statePublisher` | x500 TF / URDF |
| Qground | `qground` | QGroundControl (`gcs`) |
| Rtabmap | `rtabmap` | SLAM (`slam`) |
| NAV2 / Nav2_Rviz | `nav2`, `nav2_rviz` | Navigation + RViz (`nav`) |

Simulation assets and startup scripts live under [includes/](includes/). See [includes/README.md](includes/README.md) for layout and GitHub automation.

Operational scripts: [scripts/README.md](scripts/README.md) (`up.sh`, `smoke_test.sh`).

## Infrastructure

### Docker

[Docker Compose](docker-compose-px4.yml) runs multiple containers on a custom bridge network (`10.20.10.0/24`). The PX4 container bundles SITL, Gazebo, Micro-XRCE-DDS agent, and the `ros_gz` bridge because ROS 2 discovery across containers requires extra DDS configuration.

Build locally:

```bash
./DockerBuild.sh dockerFile/Dockerfile_px4_sim_NO_GPU local
```

### X11

GUI apps (QGroundControl, RViz) need X11 forwarding:

```bash
sudo apt-get install xauth xorg openbox
xhost +local:
```

### PX4

[PX4 v1.17](https://docs.px4.io/v1.17/en/) with Gazebo Harmonic simulation. Custom models and worlds are overlaid at container start via [includes/gz/gz_modifications.bash](includes/gz/gz_modifications.bash).

### QGroundControl

Add a UDP comm link in QGC pointing at the PX4 container hostname `px4_sim` on ports `14550`, `14540`, `14580`, `18570`. MAVLink ports are visible in:

```bash
docker logs px4_sim 2>&1 | grep mavlink
```

### ROS 2 and ros_gz bridge

Bridge config: [includes/gz/config_gz_bridge.yaml](includes/gz/config_gz_bridge.yaml)

CycloneDDS is pre-installed in the image (`ros-jazzy-rmw-cyclonedds-cpp`).

## Camera and vision

The simulated camera is a Luxonis OAK-D S2. Gazebo still loads it as `model://OakD-Lite`, because that is the name PX4's `x500_depth` includes. `CameraType=stereo` swaps in the OV9282 pair. `CameraType=rgbd` swaps in the IMX378 color camera with depth aligned to that same optical frame.

`includes/gz/oakd_s2/geometry.py` is the only copy of the intrinsics, the 7.5 cm baseline, and the mount. Container start renders the SDF and the URDF from it, and patches the `x500_depth` include so Gazebo and TF use one pose. Defaults match PX4: `0.12 0.03 0.242` plus 17 degrees down. Set `CAM_PITCH_DEG`, `CAM_X`, `CAM_Y`, and `CAM_Z` in `.env`, then recreate the PX4 and StatePublisher containers.

Stereo RTAB-Map reads `/camera/stereo/right/camera_info_baseline`. Both cameras render the left calibration, and that topic carries the same K with `P[3] = -fx * 0.075`. `P[3] / P[0]` stays `-0.075` at every allowed stereo size, because fx scales with the width. Both launches use `use_sim_time:=true`, `frame_id:=base_link`, and `imu_topic:=/imu`. Stereo uses exact sync. RGB-D stays on approximate sync because color and depth are two sensors. The viewer flag is the launch argument `rtabmap_viz`, driven by `RTABMAPVIZ` (default false).

`VISION_PROFILE` selects the stereo size, the sensor rate, and the RTAB-Map ini (`includes/gz/startFiles/rtabmap_profiles/`). The default is `full`. `CAM_STEREO_RES` (or `CAM_STEREO_WIDTH` with `CAM_STEREO_HEIGHT`) may only be `1280x800`, `640x400`, or `320x200`. `CAM_RATE_HZ` overrides the camera rate. Both `CameraType=stereo` and `CameraType=rgbd` use the same profile.

| Profile | Stereo | Rate | Intended machine |
|---|---|---|---|
| `cpu` | 320x200 | 10 Hz | CPU-only simulation and CI. Not for a real OAK-D. |
| `full` | 1280x800 | 30 Hz | Simulation on a machine with a GPU. |
| `hw` | 1280x800 | 15 Hz | Real OAK-D S2 on a LattePanda or Jetson Orin NX. RGB-D is the light host path, because the camera computes depth. |

On an 8-core CPU-only machine, before `full` moved to 1280x800: `full` at 640x400 measured RTF 0.125, with stereo odometry at 30.3 Hz in sim time and 3.8 Hz on the wall (64 ms per frame). `cpu` measured RTF 0.43-0.46 and 10.0 Hz in sim time (4.3-4.6 Hz on the wall, 25-35 ms per frame). `CAM_RATE_HZ=15` on `cpu` measured RTF 0.31 and 15.2 Hz in sim time. Parameter files and the odometry checks are in [docs/rtabmap_tuning.md](docs/rtabmap_tuning.md).

Frame-by-frame explanation, including what used to disagree: [docs/frames.md](docs/frames.md). Generator details: [includes/gz/oakd_s2/README.md](includes/gz/oakd_s2/README.md).

```bash
# Odometry gates as JSONL. --fail-on-loss exits 1 when a VISION_* threshold is breached.
python3 HealthCheck/rtabmap_health_log.py --output /tmp/rtabmap_health.jsonl

# Topics, baseline Tx, IMU, and TF. Needs a running stack. Not run in CI yet.
python3 HealthCheck/check_vision_pipeline.py --mode stereo --output /tmp/vision_check.jsonl

# Camera, IMU, odom, and /clock real-time factor for 10 wall seconds. Needs a running stack.
python3 HealthCheck/vision_rate_probe.py --seconds 10 --output /tmp/vision_rate.jsonl

# Any two TUM files. -a only, so scale error is not hidden.
python3 scripts/eval_slam_accuracy.py ground_truth.tum rtabmap.tum
```

`SLAM_APE_RMS_MAX` and `SLAM_DRIFT_PER_M_MAX` are the gates for that last script. The checked-in defaults are starting values, not a number measured on a flight.

`VISION_MAX_LOST_STREAK`, `VISION_MAX_RECOVERY_FRAMES`, `VISION_MIN_MEDIAN_FEATURES`, `VISION_MIN_INLIERS`, and `VISION_MIN_ODOM_HZ` are the gates for the health log. Feature and inlier floors follow the profile unless those two variables are set. `cpu` is 40 and 15, measured at 320x200. `full` is 120 and 20: 120 was measured at 640x400 and is not yet remeasured at 1280x800. `hw` is 80 and 20, not yet measured. The summary line has each metric, its threshold, and pass/fail, plus the wall-clock odometry rate next to the sim-time rate.

## Health checks

The PX4 service healthcheck verifies `/clock` and `/fmu/out/vehicle_odometry`. RTAB-Map checks `/rtabmap/odom`. Scripts live in [HealthCheck/](HealthCheck/).

## CI

[.github/workflows/docker-image.yml](.github/workflows/docker-image.yml) builds NO-GPU and GPU images on Dockerfile changes, pushes versioned tags, and runs a headless SITL smoke test on the NO-GPU image.

## Deferred work

The following are planned but not in scope for the current non-GPU release:

- **GPU compose** — [docker-compose-px4-GPU.yml](docker-compose-px4-GPU.yml) needs YAML/env fixes and NVIDIA runtime configuration
- **Image slimming** — multi-stage builds and optional minimal image without Nav2/RTAB-Map/QGC
- **Vagrant host** — [vagrant/](vagrant/) still targets ROS Humble on Ubuntu 22.04

## Legacy

- Gazebo Classic assets: [includes/gazebo_classic/](includes/gazebo_classic/)
- [DockerRun.sh](DockerRun.sh) — legacy single-container workflow; use `scripts/up.sh` instead
- [includes/gz/CMakeLists.txt](includes/gz/CMakeLists.txt) — legacy PX4 1.15 overlay; PX4 1.17 discovers worlds via upstream `gz_bridge` CMake GLOB after copying SDF files
