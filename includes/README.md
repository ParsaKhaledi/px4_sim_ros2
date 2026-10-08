# includes/

Runtime simulation assets mounted into Docker containers from the host repo. Changes here take effect on **container recreate** without rebuilding the image (unless you change something baked into the Dockerfile).

## Layout

| Path | Purpose |
|------|---------|
| [gz/](gz/) | **Active** Gazebo Harmonic stack: worlds, models, bridge config, Nav2 params, startup scripts |
| [gazebo_classic/](gazebo_classic/) | Legacy Gazebo Classic assets (not used by current `docker-compose-px4.yml`) |

### gz/ (used at runtime)

| Path | Used by |
|------|---------|
| `gz_modifications.bash` | PX4 container — patches camera model and copies custom worlds/models into PX4-Autopilot |
| `worlds/*.sdf` | World selection via `World=<filename_stem>` |
| `models/` | Custom GZ models (Oak-D rgbd/stereo, apt, furniture, etc.) |
| `config_gz_bridge.yaml` | ros_gz bridge topic mapping |
| `Params/nav2/` | Nav2 and RViz configuration |
| `startFiles/` | Per-service launch scripts referenced from Compose |

Mount points in Compose (example): `./includes/gz/` → `/home/px4/volume/includes/gz/`, `./includes/gz/startFiles/` → `/home/px4/volume/startFiles/`.

## GitHub automation

CI is defined in [.github/workflows/docker-image.yml](../.github/workflows/docker-image.yml).

### What triggers CI

| Change | Triggers image build? |
|--------|------------------------|
| `dockerFile/**` | Yes |
| `.github/workflows/docker-image.yml` | Yes |
| `scripts/smoke_test.sh` | Yes |
| `includes/**` | **No** (runtime bind-mount only) |

Pushes to `main` or `docker` run the workflow (or use **Actions → Run workflow** manually).

### What CI does

1. **Build matrix** — builds and pushes two images to Docker Hub (`alienkh/px4_sim`):
   - NO-GPU: `dockerFile/Dockerfile_px4_sim_NO_GPU`
   - GPU: `dockerFile/Dockerfile_px4_sim_with_GPU` (tag suffix `_GPU`)

2. **Tags pushed** — `v3.0.0`, `v3.0.0-latest` (and `_GPU` variants). Build cache is stored in GitHub Actions cache (not Docker Hub).

3. **Smoke test** — after NO-GPU push, runs [scripts/smoke_test.sh](../scripts/smoke_test.sh) (see [scripts/README.md](../scripts/README.md)) which:
   - Pulls the new image
   - Mounts `includes/gz/` and `HealthCheck/` from this repo
   - Runs `gz_modifications.bash`, SITL, bridge, and XRCE headlessly
   - Asserts ROS topics `/clock` and `/fmu/out/vehicle_odometry` are publishing

### Secrets required

- `DOCKER_USERNAME`
- `DOCKER_PASSWORD`

### After CI

Update `px4TAG` in your local `.env` to the new tag (e.g. `v3.0.0`) and `docker compose pull`.

### Planned / future automation

- CI on changes under `includes/gz/` (config-only smoke test, no full image rebuild)
- Validate Compose and startup scripts on pull requests
- Optional workflow to bump `px4TAG` in `.env.example` when a build succeeds

## Local workflow vs CI

| Task | Rebuild image? | Recreate containers? |
|------|----------------|----------------------|
| Edit world/model in `includes/gz/` | No | Yes (`docker compose up -d --force-recreate PX4`) |
| Edit `startFiles/` or `Params/` | No | Yes (service that uses the script) |
| Edit Dockerfile / PX4 version | Yes (`./DockerBuild.sh` or wait for CI) | Yes (pull new tag) |
