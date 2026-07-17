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
| `FlightHeight` | Offboard takeoff height in metres (`mission` profile) | `2` |

CI publishes tags like `v3.0.0` and `v3.0.0-latest` (GPU: `v3.0.0_GPU`, `v3.0.0-latest_GPU`). Update `px4TAG` in `.env` after pulling a new build.

### Compose profiles

| Profile | Services |
|---------|----------|
| *(none)* | PX4, StatePublisher |
| `gcs` | QGroundControl |
| `slam` | RTAB-Map |
| `nav` | Nav2 + RViz (use with `slam`) |
| `mission` | Offboard (px4_imu_bridge + microxrce_offboard; arms and flies — use with `slam`, add `nav` for closed-loop autonomy) |

```bash
# Simulation + ground station only
COMPOSE_PROFILES=gcs CameraType=rgbd World=apt_world ./scripts/up.sh

# Full robotics stack
COMPOSE_PROFILES=gcs,slam,nav CameraType=rgbd World=default ./scripts/up.sh

# Stereo camera in a custom world
COMPOSE_PROFILES=gcs CameraType=stereo World=husarion_office ./scripts/up.sh

# Closed-loop autonomous flight in the indoor apartment world
COMPOSE_PROFILES=slam,nav,mission CameraType=rgbd World=apt_world FlightHeight=1.5 ./scripts/up.sh
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

**Profile dependencies:** `nav` depends on `slam` (Nav2 waits for Rtabmap); `mission` also depends on `slam` (Offboard waits for Rtabmap, since it consumes `/rtabmap/odom`). Use `COMPOSE_PROFILES=slam,nav,mission` or include the profiles you need. Services without a profile (`PX4`, `StatePublisher`) always start.

**Without `up.sh`:** `CameraType=rgbd World=default docker compose -f docker-compose-px4.yml --profile gcs up -d`

**Display / X11:** QGC and RViz need `DISPLAY` and `xhost +local:`. Set `XAUTH` if your setup uses `/tmp/.docker.xauth` (Compose may warn if unset).

| Service | Container | Role |
|---------|-----------|------|
| PX4 | `px4_sim` | SITL, Gazebo, XRCE agent, ros_gz bridge |
| StatePublisher | `statePublisher` | x500 TF / URDF |
| Qground | `qground` | QGroundControl (`gcs`) |
| Rtabmap | `rtabmap` | SLAM (`slam`) |
| NAV2 / Nav2_Rviz | `nav2`, `nav2_rviz` | Navigation + RViz (`nav`) |
| Offboard | `offboard` | PX4 IMU bridge + offboard flight control (`mission`) |

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

CycloneDDS is pre-installed in the image (`ros-jazzy-rmw-cyclonedds-cpp`) but **not** activated by default (the ROS 2 default `rmw_fastrtps_cpp` is used). To opt in, uncomment the `RMW_IMPLEMENTATION: rmw_cyclonedds_cpp` line in every service's `environment:` block in [docker-compose-px4.yml](docker-compose-px4.yml) — it must be consistent across all services or nodes won't discover each other.

### Offboard control and IMU source (`mission` profile)

[includes/gz/px4_imu_bridge.py](includes/gz/px4_imu_bridge.py) subscribes to PX4's own uXRCE-DDS topics — `/fmu/out/sensor_combined` (gyro/accelerometer) and `/fmu/out/vehicle_attitude` (orientation) — converts PX4's FRD body / NED world convention to ROS's FLU body / ENU world convention, and republishes `sensor_msgs/Imu` on `/imu` (`frame_id: base_link`). This is what RTAB-Map's `imu_topic:=/imu` consumes for both `rgbd` and `stereo` camera types; the OakD model's own `imu/data` sensor is left defined but unused. [includes/gz/microxrce_offboard.py](includes/gz/microxrce_offboard.py) consumes RTAB-Map's `/rtabmap/odom` (VIO) and Nav2's `/cmd_vel` (`geometry_msgs/TwistStamped`), and drives PX4 offboard arm/takeoff/velocity control.

### Time synchronization

The whole stack runs on simulation time: every node (bridge, offboard, RTAB-Map, Nav2, RViz, robot_state_publisher) is launched with `use_sim_time:=true`, and PX4's own DDS time sync is disabled (`UXRCE_DDS_SYNCT 0` in [gz_modifications.bash](includes/gz/gz_modifications.bash)) so Gazebo's `/clock` is the single time source. This is PX4's documented "Gazebo-clock scenario" for perception-heavy sims where the simulation doesn't run at real-time factor 1.

## Health checks

The PX4 service healthcheck verifies `/clock` and `/fmu/out/vehicle_odometry`. RTAB-Map checks `/rtabmap/odom`. Scripts live in [HealthCheck/](HealthCheck/).

## CI

[.github/workflows/docker-image.yml](.github/workflows/docker-image.yml) builds NO-GPU and GPU images on Dockerfile changes, pushes versioned tags, and runs a headless SITL smoke test on the NO-GPU image.

## Deferred work

The following are planned but not in scope for the current non-GPU release:

- **GPU compose** — [docker-compose-px4-GPU.yml](docker-compose-px4-GPU.yml) needs YAML/env fixes and NVIDIA runtime configuration. The GPU [Dockerfile](dockerFile/Dockerfile_px4_sim_with_GPU) builds and includes the same Python deps as the NO-GPU image, but the closed-loop offboard-flight stack (IMU bridge + offboard control + RTAB-Map/Nav2) is **not fully tested** on it — only the NO-GPU image has been validated end-to-end.
- **Image slimming** — multi-stage builds and optional minimal image without Nav2/RTAB-Map/QGC
- **Vagrant host** — [vagrant/](vagrant/) still targets ROS Humble on Ubuntu 22.04

## Legacy

- Gazebo Classic assets: [includes/gazebo_classic/](includes/gazebo_classic/)
- [DockerRun.sh](DockerRun.sh) — legacy single-container workflow; use `scripts/up.sh` instead
- [includes/gz/CMakeLists.txt](includes/gz/CMakeLists.txt) — legacy PX4 1.15 overlay; PX4 1.17 discovers worlds via upstream `gz_bridge` CMake GLOB after copying SDF files
