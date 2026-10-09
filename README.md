# PX4 Simulation with ROS 2

Docker-orchestrated PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy simulation stack. Pull a pre-built image from [Docker Hub](https://hub.docker.com/r/alienkh/px4_sim) and launch with Docker Compose.

## Version Matrix

| Component | Version |
|-----------|---------|
| PX4-Autopilot | v1.17.0 (`versions.env`) |
| px4_msgs | [v1.17.0](https://github.com/PX4/px4_msgs/releases/tag/v1.17.0) (`PX4_MSGS_REF`) |
| px4_ros_com | main (examples only, unpinned) |
| ROS 2 | Jazzy (`ROS_DISTRO` in `versions.env`) |
| Gazebo | Harmonic (`gz-harmonic`) |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Micro-XRCE-DDS-Agent | v2.4.3 (`XRCE_AGENT_VERSION`) |

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
| `registry` / `PX4_IMAGE` | Image to run | `docker.io/alienkh/px4_sim:1.17.0_121` |
| `px4TAG` | Tag inside `PX4_IMAGE` | `1.17.0_121` |
| `PX4_GZ_MODEL_POSE` | Spawn pose `x,y,z,roll,pitch,yaw`. z is drop clearance | `-3,-1.6,0.15,0,0,3.14` |
| `CameraType` | `rgbd` or `stereo` | `rgbd` |
| `CAM_PITCH_DEG` | OAK-D pitch, degrees, positive lens-down | `17` |
| `CAM_X`, `CAM_Y`, `CAM_Z` | OAK-D mount on the x500, meters | `0.12`, `0.03`, `0.242` |
| `VISION_PROFILE` | Camera profile. `cpu` is the one this line runs | `cpu` |
| `IMU_SOURCE` | Camera IMU on `/imu` | `oak` |
| `World` | Gazebo world filename stem | `default` |
| `HEADLESS` | `1` skips the Gazebo GUI | `0` |
| `RTABMAPVIZ` | RTAB-Map visualization. Unset stays closed. | `false` |
| `COMPOSE_PROFILES` | Comma-separated profiles | `gcs,slam,nav` |

Component versions (PX4, px4_msgs, XRCE agent, ROS distro) live in [versions.env](versions.env). `.env` only pins the image you pull and the runtime knobs.

Pushes to `main` and `v*` tags publish `px4-1.17.0` and `sha-<short>` (GPU: `px4-1.17.0-gpu`). Pull requests build the CPU image only when a Dockerfile, `versions.env`, `includes/gz/patch_dds_topics.py`, or a `ros2_ws` package manifest changed. Otherwise CI pulls `PX4_IMAGE` (`1.17.0_121`). The flight job loads that same image and does not build it again. The GPU image is built on `main`, `v*` tags, and a manual workflow run, and that job never flies. No `1.17` GPU tag is published. `px4GPUTAG` is the name a local GPU build uses. `COMPOSE_PROJECT_NAME` selects the Compose project so two stacks can run on one host and `down` removes only that project.

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

# Stereo camera in the apartment
COMPOSE_PROFILES=gcs CameraType=stereo World=apt_world ./scripts/up.sh
```

You can also set `CameraType` and `World` in `.env` instead of the command line.

### Valid worlds

World names match SDF **filenames** under `includes/gz/worlds/` (without `.sdf`):

- `default` — PX4 default world
- `apt_world`

### Runtime configuration (brief)

**How `CameraType` and `World` are applied:** `scripts/up.sh` exports both variables into Compose. The PX4 container runs `gz_modifications.bash` (camera + custom models/worlds) then `gz_start_px4_gz_sim.sh` (world/make target). If the `slam` profile is active, Rtabmap receives the same `CameraType`.

**Changing world or camera on a running stack:** These are applied at container start. Bring the stack down first, or recreate PX4:

```bash
docker compose -f docker-compose-px4.yml down
CameraType=stereo World=apt_world ./scripts/up.sh
# or: CameraType=stereo World=apt_world docker compose -f docker-compose-px4.yml up -d --force-recreate PX4
```

**Verify:** `docker compose -f docker-compose-px4.yml logs PX4 2>&1 | grep "Selected Camera Type"`

**Profile dependencies:** `nav` depends on `slam` (Nav2 waits for Rtabmap). Use `COMPOSE_PROFILES=slam,nav` or include both. Services without a profile (`PX4`, `StatePublisher`) always start.

**Without `up.sh`:** `CameraType=rgbd World=default docker compose -f docker-compose-px4.yml --profile gcs up -d`

**Display / X11:** QGC and RViz need `DISPLAY` and `xhost +local:`. Set `XAUTH` if your setup uses `/tmp/.docker.xauth` (Compose may warn if unset).

| Service | Alias | Role |
|---------|-------|------|
| PX4 | `px4_sim` | SITL, Gazebo, XRCE agent, ros_gz bridge |
| StatePublisher | `statePublisher` | x500 TF / URDF |
| Qground | `qground` | QGroundControl (`gcs`) |
| Rtabmap | `rtabmap` | SLAM (`slam`) |
| NAV2 / Nav2_Rviz | `nav2`, `nav2_rviz` | Navigation + RViz (`nav`) |

Simulation assets and startup scripts live under [includes/](includes/). See [includes/README.md](includes/README.md) for layout. Headless Gazebo, sim time, ground truth, and preflight checks are described in [docs/simulation.md](docs/simulation.md). `/imu` is the camera IMU (`IMU_SOURCE=oak`).

Operational scripts: [scripts/README.md](scripts/README.md).

## Infrastructure

### Docker

[Docker Compose](docker-compose-px4.yml) runs one project per `COMPOSE_PROJECT_NAME`. Docker assigns the bridge subnet, so a second project can start beside the first. The old container names stay as network aliases (`px4_sim`, `statePublisher`, `qground`, `rtabmap`, `nav2`, `nav2_rviz`). The PX4 service bundles SITL, Gazebo, Micro-XRCE-DDS agent, and the `ros_gz` bridge because ROS 2 discovery across containers requires extra DDS configuration.

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

Add a UDP comm link in QGC pointing at the PX4 hostname `px4_sim` on ports `14550`, `14540`, `14580`, `18570`. MAVLink ports are visible in:

```bash
docker compose -f docker-compose-px4.yml logs PX4 2>&1 | grep mavlink
```

### ROS 2 and ros_gz bridge

Bridge config: [includes/gz/config_gz_bridge.yaml](includes/gz/config_gz_bridge.yaml)

CycloneDDS is pre-installed in the image (`ros-jazzy-rmw-cyclonedds-cpp`).

## Health checks

[HealthCheck/healthcheck.py](HealthCheck/healthcheck.py) subscribes to each service's topics and writes JSONL under `logs/health/`. Compose marks a service healthy only when the required rates are met, and later services wait on `service_healthy`.

```bash
./scripts/health_report.sh
```

`/ground_truth/odom` is checked at >= 45 Hz only when something is already publishing it. Camera topics follow `CameraType`.

## CI and local tests

Pull requests run lint, unit tests (pytest and colcon), one image build, and one headless hover/smoke on that image. The flight job loads the image from the build job and runs `scripts/smoke_test.sh` with `docker-compose-px4.yml` and the headless `compose.ci.yml` override. It uses the plain `x500` (`CameraType=none`). It does not run an offboard controller. Nothing is pushed from a pull request.

Copy `.env.example` to `.env` for a local run. `.env` is not committed.

```bash
# Static checks
./scripts/check_versions.sh
docker compose -f docker-compose-px4.yml config -q
docker compose -f docker-compose-px4.yml -f compose.ci.yml config -q

# The one headless hover/smoke.
HEADLESS=1 RTABMAPVIZ=false CameraType=none PX4_GZ_MODEL=x500 \
  COMPOSE_SERVICES=PX4 ./scripts/smoke_test.sh "${PX4_IMAGE}"
```

See [scripts/README.md](scripts/README.md).

## Deferred work

- **Image slimming** — multi-stage builds and a runtime image without the CUDA devel toolchain
- **QGroundControl digest pin** — the AppImage URL is still a moving channel (`versions.env`)
- **px4_ros_com** — still cloned from `main`
- **Fuel meshes** — `husarion_office.sdf` still references fuel.gazebosim.org (CI warns, it does not fail)
- **Vagrant host** — [vagrant/](vagrant/) still targets ROS Humble on Ubuntu 22.04

## Legacy

- Gazebo Classic assets: [includes/gazebo_classic/](includes/gazebo_classic/)
- [DockerRun.sh](DockerRun.sh) — legacy single-container workflow; use `scripts/up.sh` instead
- [includes/gz/CMakeLists.txt](includes/gz/CMakeLists.txt) — legacy PX4 1.15 overlay; PX4 1.17 discovers worlds via upstream `gz_bridge` CMake GLOB after copying SDF files
