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
| `VISION_PROFILE` | `full` (30 Hz cameras, 200 Hz IMU) or `cpu` (10 Hz cameras, 100 Hz IMU) | `full` |
| `IMU_SOURCE` | `oak` (camera IMU) or `px4` (flight IMU on `/imu`) | `oak` |
| `World` | Gazebo world filename stem | `default` |
| `COMPOSE_PROFILES` | Comma-separated profiles | `gcs,slam,nav` |

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

Simulation assets and startup scripts live under [includes/](includes/). See [includes/README.md](includes/README.md) for layout and GitHub automation. Headless Gazebo, sim time, ground truth, offline worlds, wall maps, trajectory scoring, and preflight checks are described in [docs/simulation.md](docs/simulation.md).

## IMU source

`IMU_SOURCE` selects the `sensor_msgs/Imu` publisher on `/imu`. RTAB-Map already subscribes to that topic. This stack does not publish static transforms for `camera_link`, the optical frames, or `imu_link`. The vision StatePublisher owns that tree, including the camera mount. Preflight still requires a TF path from the `/imu` `frame_id` to the camera optical frame.

| | `oak` (default) | `px4` |
| --- | --- | --- |
| When | The camera IMU is on the Oak-D, so no lever arm | EKF2 and RTAB-Map should share the flight IMU |
| Publisher | Gazebo `/imu` bridged to ROS `/imu` | `px4_imu_relay` from `/fmu/out/sensor_combined` and `/fmu/out/vehicle_attitude` |
| `frame_id` | `imu_link` | `base_link` |
| Preflight minimum | half of `IMU_RATE_HZ`, or half the `VISION_PROFILE` IMU rate | 80 Hz in sim time, unless `PREFLIGHT_MIN_IMU_HZ` is set |

`px4` stamps each sample with ROS time when it is received (`IMU_STAMP_MODE=receive`, the default). With `use_sim_time` that clock is Gazebo `/clock`. `IMU_STAMP_MODE=px4_offset` adds one fixed offset, measured once at startup, from the PX4 sample time to `/clock`. SITL lockstep keeps PX4's clock on Gazebo time, so that constant stays valid. `sensor_combined` in px4_msgs v1.17 has `timestamp` and no `timestamp_sample`; the relay uses `timestamp_sample` when the message has it. A raw PX4 boot timestamp is not published. RTAB-Map's `wait_imu_to_init` compares IMU stamps with image stamps and would drop those samples.

Orientation is roll and pitch after the full NED/FRD to ENU/FLU conversion, with yaw set to zero. `orientation_covariance` is diagonal: roll and pitch use `orientation_stddev` squared (default 0.02 rad), and yaw uses `yaw_variance` (default 1e3 rad²). RTAB-Map then takes gravity from the quaternion and ignores heading. That avoids a yaw feedback loop when EKF2's yaw comes from RTAB-Map in vision mode.

Gyro and specific force stay in the body frame and map FRD to FLU as `(x, -y, -z)`. At rest the z acceleration is about +9.81 m/s².

The relay logs the measured `sensor_combined` input rate in sim time and warns below `rate_warn_hz` (default 80). XRCE copies that topic at most once per 10 ms, so 100 Hz is a ceiling and a 100 Hz gate would fail on ordinary jitter. RTAB-Map's IMU path still wants 100 Hz or more. The `px4` preflight minimum is that same 80 Hz when `PREFLIGHT_MIN_IMU_HZ` is unset.

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
