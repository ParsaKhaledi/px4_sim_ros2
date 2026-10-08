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

CI is defined in [.github/workflows/ci.yml](../.github/workflows/ci.yml). Component versions come from [versions.env](../versions.env).

### What triggers CI

Pull requests and pushes to `main` or `dev/px4-upgrade` that touch Dockerfiles, Compose, `scripts/`, `HealthCheck/`, `ros2_ws/`, `tests/`, or the workflow files. `includes/gz/**` is bind-mounted at runtime, so editing a world does not rebuild the image. The spawn-pose and RTAB-Map viz hooks under `includes/gz/startFiles/` are on the path filter.

Pull requests do not push images. A push to `main` (or a `v*` tag) publishes `px4-<PX4_VERSION>` and `sha-<short>`.

### What CI does

1. **Lint** — hadolint, shellcheck, yamllint, actionlint, `docker compose config`, version pins, and a warn-only Fuel URI check.
2. **colcon** — builds `ros2_ws`. An empty `src/` is a successful no-op until the control packages arrive.
3. **Image** — builds the no-GPU Dockerfile once, records the digest, and runs `scripts/image_assert.sh` on that image. The GPU Dockerfile builds on `main` and on manual dispatch. Gazebo smoke stays on the nightly/manual workflow because hosted runners have no GPU.

### Secrets required

- `DOCKER_USERNAME`
- `DOCKER_PASSWORD` (used only when the workflow pushes)

### After CI

Update `PX4_IMAGE` in `.env` if you want Compose to pull the new tag, then `docker compose pull`.

## Local workflow vs CI

| Task | Rebuild image? | Recreate containers? |
|------|----------------|----------------------|
| Edit world/model in `includes/gz/` | No | Yes (`docker compose up -d --force-recreate PX4`) |
| Edit `startFiles/` or `Params/` | No | Yes (service that uses the script) |
| Edit Dockerfile / PX4 version | Yes (`./DockerBuild.sh` or wait for CI) | Yes (pull new tag) |
