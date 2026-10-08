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
| `registry` / `PX4_IMAGE` | Image to run | `docker.io/alienkh/px4_sim:1.17.0_01` |
| `px4TAG` | Tag inside `PX4_IMAGE` | `1.17.0_01` |
| `PX4_GZ_MODEL_POSE` | Spawn pose `x,y,z,roll,pitch,yaw` | `-3,-1.6,0,0,0,3.14` |
| `CameraType` | `rgbd` or `stereo` | `rgbd` |
| `CAM_PITCH_DEG` | OAK-D pitch, degrees, positive lens-down | `17` |
| `CAM_X`, `CAM_Y`, `CAM_Z` | OAK-D mount on the x500, meters | `0.12`, `0.03`, `0.242` |
| `World` | Gazebo world filename stem | `default` |
| `HEADLESS` | `1` skips the Gazebo GUI | `0` |
| `RTABMAPVIZ` | RTAB-Map visualization. Unset stays closed. | `false` |
| `COMPOSE_PROFILES` | Comma-separated profiles | `gcs,slam,nav` |

Component versions (PX4, px4_msgs, XRCE agent, ROS distro) live in [versions.env](versions.env). `.env` only pins the image you pull and the runtime knobs.

Pushes to `main` and `v*` tags publish `px4-1.17.0` and `sha-<short>` (GPU: `px4-1.17.0-gpu`). Pull requests build the CPU image only and do not push it. The GPU image is built on `main`, `v*` tags, and a manual workflow run, and that job never flies. Point `PX4_IMAGE` at a published tag when you want to run that build.

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

**GPU:** `GPU=1 ./scripts/up.sh` or `docker compose -f compose.yml -f compose.gpu.yml -f compose.gui.yml up -d`. [docker-compose-px4-GPU.yml](docker-compose-px4-GPU.yml) is the same stack with the NVIDIA image and device reservation.

**Display / X11:** QGC and RViz need `DISPLAY` and `xhost +local:`. Set `XAUTH` if your setup uses `/tmp/.docker.xauth` (Compose may warn if unset).

| Service | Container | Role |
|---------|-----------|------|
| PX4 | `px4_sim` | SITL, Gazebo, XRCE agent, ros_gz bridge |
| StatePublisher | `statePublisher` | x500 TF / URDF |
| Qground | `qground` | QGroundControl (`gcs`) |
| Rtabmap | `rtabmap` | SLAM (`slam`) |
| NAV2 / Nav2_Rviz | `nav2`, `nav2_rviz` | Navigation + RViz (`nav`) |

Simulation assets and startup scripts live under [includes/](includes/). See [includes/README.md](includes/README.md) for layout and GitHub automation.

Operational scripts: [scripts/README.md](scripts/README.md).

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

[HealthCheck/healthcheck.py](HealthCheck/healthcheck.py) subscribes to each service's topics and writes JSONL under `logs/health/`. Compose marks a service healthy only when the required rates are met, and later services wait on `service_healthy`.

```bash
./scripts/health_report.sh
```

`/ground_truth/odom` is checked at >= 45 Hz only when something is already publishing it. Camera topics follow `CameraType`.

## CI and local tests

Pull requests run lint, colcon, the CPU image build, and two headless flights on that image. Nothing is pushed from a pull request. The GPU image is not built on a pull request. There is no GPU flight.

The fast flight uses the plain `x500` (`CameraType=none`, `PX4_GZ_MODEL=x500`). The second flight uses `x500_depth` with `CameraType=rgbd` and renders on the CPU through Mesa llvmpipe (`LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`). Gazebo starts with `--headless-rendering` (`GZ_HEADLESS_RENDERING=1`). If the rgb or depth frames are missing, or flat (variance under `E2E_CAMERA_MIN_VARIANCE`, default 1), the script recreates the sim on Xvfb (`GZ_USE_XVFB=1`, [compose.xvfb.yml](compose.xvfb.yml)). Expect a real-time factor around 0.3–0.6. PX4 lockstep keeps the mission valid; `E2E_WALL_SCALE=3` stretches the wall-clock timeouts. Both flights share one image load inside the `flight_test` job, because a second job would build the image again.

Each flight takes off to 2 m, hovers 10 s, flies `E2E_LEG_LENGTH_M` (0.3 m), yaws 180°, flies back, then lands. The hover clock starts only after height has held ±5 cm for 2 s. Each leg and the yaw wait until they settle: position inside ±2 cm for 1 s, and that hold finishing within 4 s of the step. Yaw settles inside ±3°. Grading uses the Gazebo track, so a step that overshoots (3 cm, or 5° in yaw) or never settles fails even if the driver moves on. Legs must finish at 30 cm ± 3 cm, and the landing must be within 5 cm of the start. The log includes the PX4 `vehicle_local_position` error against the Gazebo pose. `logs/flights/*/trajectory.json` also stores the Gazebo real-time factor and, for the camera flight, each camera topic's rate, mean, and variance. If `px4_control.Drone` imports, that API flies the same mission. The grade is the same either way.

A crash (tilt past about 60°, a ground impact, an unexpected disarm, failsafe, or land, pose far from the setpoint, or a few seconds without odometry) records the reason and restarts the PX4 container. `E2E_MAX_RETRIES` (default 2) is how many restarts are allowed after the first try. The run fails when every attempt crashes. Thresholds are the `E2E_*` keys in `.env`.

Nightly keeps the 1.0 m leg and the depth-camera model (`x500_depth`) on a self-hosted runner.

```bash
# Static checks
./scripts/check_versions.sh
docker compose -f docker-compose-px4.yml config -q
docker compose -f docker-compose-px4-GPU.yml config -q

# Headless smoke against a local or pulled image.
HEADLESS=1 RTABMAPVIZ=false ./scripts/smoke_test.sh "${PX4_IMAGE}"

# Fast out-and-back, same shape as the first pull-request flight.
CameraType=none PX4_GZ_MODEL=x500 COMPOSE_SERVICES=PX4 ./scripts/run_e2e.sh

# Camera model on Mesa llvmpipe. Software RTF is well below 1.
CameraType=rgbd PX4_GZ_MODEL=x500_depth GZ_HEADLESS_RENDERING=1 \
  E2E_CHECK_CAMERAS=1 E2E_WALL_SCALE=3 COMPOSE_SERVICES=PX4 ./scripts/run_e2e.sh

# Nightly shape: depth camera and a 1 m leg, on a machine that can render.
CameraType=rgbd PX4_GZ_MODEL=x500_depth E2E_LEG_LENGTH_M=1.0 ./scripts/run_e2e.sh
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
