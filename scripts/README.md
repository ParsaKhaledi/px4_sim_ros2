# scripts/

Helper scripts for launching the simulation stack locally and validating Docker images in CI.

| Script | Purpose |
|--------|---------|
| [up.sh](up.sh) | Start the Compose stack with profiles, camera, and world selection |
| [smoke_test.sh](smoke_test.sh) | Headless SITL health check for a given image (used in GitHub Actions) |

---

## up.sh

Starts [docker-compose-px4.yml](../docker-compose-px4.yml) from the repo root with sensible defaults.

### Prerequisites

- Docker and Docker Compose installed
- `.env` file (copy from [.env.example](../.env.example)) with `registry` and `px4TAG`
- For GUI services (`gcs`, `nav`): X11 and `xhost +local:` (the script runs `xhost` automatically when `DISPLAY` is set)

### Usage

```bash
# Full stack (default profiles: gcs, slam, nav)
CameraType=rgbd World=default ./scripts/up.sh

# Core simulation only (PX4 + StatePublisher)
COMPOSE_PROFILES= ./scripts/up.sh

# Sim + QGroundControl in a custom world
COMPOSE_PROFILES=gcs CameraType=rgbd World=apt_world ./scripts/up.sh

# Stereo camera, full robotics stack
COMPOSE_PROFILES=gcs,slam,nav CameraType=stereo World=husarion_office ./scripts/up.sh
```

### Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `CameraType` | `rgbd` | `rgbd` or `stereo` — passed to PX4 and Rtabmap |
| `World` | `default` | SDF filename stem under `includes/gz/worlds/` |
| `COMPOSE_PROFILES` | `gcs,slam,nav` | Comma-separated Compose profiles (empty = core only) |
| `COMPOSE_FILE` | `docker-compose-px4.yml` | Alternate compose file path |

Values from `.env` (`registry`, `px4TAG`) are sourced automatically. `CameraType` and `World` can also be set in `.env` or on the command line (command line wins).

### Compose profiles

| Profile | Services started |
|---------|------------------|
| *(none)* | `PX4`, `StatePublisher` (always) |
| `gcs` | QGroundControl |
| `slam` | RTAB-Map |
| `nav` | Nav2 + RViz — **requires `slam`** |

### Changing world or camera

Settings apply when the **PX4 container starts**. To switch after a run:

```bash
docker compose -f docker-compose-px4.yml down
CameraType=stereo World=apt_world ./scripts/up.sh
```

### Verify

```bash
docker logs px4_sim 2>&1 | grep "Selected Camera Type"
docker compose -f docker-compose-px4.yml ps
```

---

## smoke_test.sh

Runs a **single privileged container** headlessly to confirm a built image can start PX4 SITL, the ros_gz bridge, and XRCE-DDS, and publish expected ROS topics.

Used by [.github/workflows/docker-image.yml](../.github/workflows/docker-image.yml) after each NO-GPU image push.

### Usage

```bash
chmod +x scripts/smoke_test.sh

# Test a local or pulled image
./scripts/smoke_test.sh alienkh/px4_sim:local

# CI example
./scripts/smoke_test.sh docker.io/alienkh/px4_sim:1.17.0_42

# Longer timeout (seconds, default 300)
SMOKE_TEST_TIMEOUT=600 ./scripts/smoke_test.sh alienkh/px4_sim:1.17.0-latest
```

### What it does

1. Removes any existing `px4_smoke_test` container
2. Starts the image with bind mounts from this repo:
   - `HealthCheck/` — topic health scripts
   - `includes/gz/` — worlds, models, `gz_modifications.bash`
   - `includes/gz/startFiles/` — SITL and bridge launch scripts
3. Inside the container (headless, `QT_QPA_PLATFORM=offscreen`):
   - `gz_modifications.bash rgbd`
   - `MicroXRCEAgent udp4 -p 8888`
   - `gz_start_px4_gz_sim.sh default`
   - `gz_start_ros2_gz_bridge.sh`
4. Every 10s, runs the same healthcheck as Compose PX4 service:
   - Topics: `/clock`, `/fmu/out/vehicle_odometry`
5. Exits `0` on success, `1` on timeout (prints last 80 log lines)
6. Removes the test container on exit (`trap cleanup EXIT`)

### When to run locally

- After `./DockerBuild.sh` before pushing an image
- When debugging CI failures on the smoke test job
- After changing `includes/gz/` startup paths or `HealthCheck/` scripts

### Limitations

- Tests **rgbd** + **default** world only (fixed in the script; not parameterized)
- Does not start QGC, Rtabmap, or Nav2
- Requires sufficient CPU/RAM; first SITL start can take several minutes
- GPU images are not smoke-tested in CI (NO-GPU image only)
