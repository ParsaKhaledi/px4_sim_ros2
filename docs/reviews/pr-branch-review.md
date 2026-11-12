# PX4 Sim ROS 2 — PR and branch review

**Audience:** maintainers deciding merge order and rebase work.  
**Mode:** explanation (why each branch exists, what the diffs do, what blocks merge).  
**Reviewed:** 2026-11-12 13:12 Asia/Tehran  
**Method:** line-level reading of GitHub PR diffs and `origin/*` tips with [Grok 4.7 medium](https://x.ai/).  
**Scope:** PRs #19–#29 plus pipeline branches `feature/line-base`, `feature/line-sim`, `feature/line-control`, and historical `feature/px4-imu-bridge-indoor-flight`.

## Verdict in one page

Nothing from this wave is on `main` yet. `origin/main` tip is still `5eb5fde` (2026-06-29). The open drafts that matter for landing work are:

| Priority | PR / branch | Role | Merge readiness |
|---|---|---|---|
| 1 | [#29](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/29) `feature/devops-compose-health-ci` | Compose, pins, healthchecks, CI smoke | Acceptable as DevOps base after renaming the “hover” step; green `flight` is boot+rates, not a commanded hover |
| 2 | Closed [#21](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/21) then [#20](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/20) | Headless GT/walls, then OAK-D S2 | Rebase onto #29; do not take their Compose files as the base |
| 3 | Vision stack [#23](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/23)→[#24](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/24)→[#25](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/25)→[#27](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/27) | Profiles, goldens, hygiene, module split | Land in that order after #20; run `test_golden.py` on the #27 tip |
| 4 | [#22](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/22) + cherry-picks from `line-control` | Offboard stack | Not ready: GPS read-back `KeyError`, `NAV_DLL_ACT` arming, Twist vs TwistStamped, vision-loss takeoff |
| 5 | [#28](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/28) / `578017e` | JSON geofence + spawn TF | Rebase after walls (#21) and control tip exist |
| 6 | [#26](https://github.com/ParsaKhaledi/px4_sim_ros2/pull/26) | Offline ULog grader | Tool-ready with synthetic tests; no real v1.17 log yet |

### How the tips relate

```
main (5eb5fde)
├── feature/devops-compose-health-ci   PR #29 open draft  (continues closed #19)
├── feature/sim-headless-groundtruth   PR #21 closed
├── feature/oakd-s2-vision             PR #20 closed
│   └── feature/vision-profiles        PR #23 closed
│       └── refactor/vision-golden-tests  PR #24 closed
│           └── refactor/vision-hygiene   PR #25 open draft
│               └── refactor/vision-modules  PR #27 open draft
├── feature/px4-control                PR #22 closed
│   └── (PR #28 geofence was based here; tip moved to line-control as 578017e)
├── feature/flight-analysis            PR #26 closed
├── feature/line-base                  DevOps CI line (+3 unique commits)
├── feature/line-sim                   sim + vision pipeline (has spawn.json writer)
├── feature/line-control               control integration (+ TwistStamped + JSON geofence)
└── feature/px4-imu-bridge-indoor-flight  superseded by px4_control
```

GitHub still names `dev/px4-upgrade` as the base for several closed PRs. That ref is gone on `origin`. The first commit parent for those PRs is `5eb5fde` (`main`). Treat them as siblings of `main`, then rebase onto #29.

### What “line by line” means here

Each section below walks the important hunks with path, line or hunk context, what the lines do, and the risk if you merge them wrong. Full mesh dumps (#21) and golden SDF blobs (#24) are summarized by contract, not every generated line.

---

## Part A — Sim, vision sensors, and DevOps

## PR review: sim, vision, and DevOps

Review of five pull requests on `ParsaKhaledi/px4_sim_ros2`. Line numbers are the new-file side of each diff (the file as it stands on that PR head). Closed PRs were not merged. `origin/dev/px4-upgrade` is not on the remote. The first commit of #19, #21, and #20 has parent `5eb5fde` (`origin/main`, “Use GHA cache and sequential pushes to fix Docker Hub uploads”). GitHub still records the base ref name `dev/px4-upgrade` for those three. Treat them as siblings of `main`, not as a stack.

How the pieces meet:

- #19 / #29 own Compose, image pins, parameter injection, and CI.
- #21 owns headless Gazebo, the spawn frame, ground truth, wall JSON, and trajectory metrics.
- #20 owns the OAK-D S2 model and RTAB-Map.
- #28 consumes #21’s wall JSON and spawn TF from the control node.

They edit the same start scripts and Compose files. A merge has to keep one `gz_start_px4_gz_sim.sh` and one Compose entrypoint.

---

### PR #19 — Pin stack versions, fix GPU compose, and add health checks

Closed, not merged. https://github.com/ParsaKhaledi/px4_sim_ros2/pull/19

#### 1. Summary

This change replaces the two hand-maintained Compose files with one service definition and small overlays. `versions.env` is the pin that `DockerBuild.sh`, both Dockerfiles, and CI read. PX4 parameters move out of `rcS` timing accidents and into `config/px4/params/sim.params`, applied in an airframe `.post` that the start script moves to before `dataman` / `ekf2`. Health checks become rate measurements in `HealthCheck/healthcheck.py`, and the PR-era workflow also builds an image and runs an out-and-back driver (`scripts/run_e2e.sh`, `tests/e2e/px4_offboard.py`). That flight is the part #29 later removes.

#### 2. Stack position

- Base ref name: `dev/px4-upgrade` (absent on `origin` now).
- Actual parent of `40fdc17`: `5eb5fde` (`origin/main`).
- Head: `feat/devops-compose-health-ci` at `b76daa572bf5ff73c277057f4dd8cf0e1883acfa`.
- 26 commits, 2026-10-08 through 2026-10-09. Closed 2026-10-09. Not merged.
- #29 is this history plus two commits, on `feature/devops-compose-health-ci`.

#### 3. Stats

82 files, +7080 / −1076.

Largest additions: `tests/e2e/px4_offboard.py` (+836), `HealthCheck/healthcheck.py` (+572), `.github/workflows/_build.yml` (+442), `scripts/run_e2e.sh` (+368), `tests/e2e/grading.py` (+346), `compose.yml` (+287), `scripts/px4_params.py` (+247). Deletions are the old Compose bodies (`docker-compose-px4.yml` −316, `docker-compose-px4-GPU.yml` −260) and the bash publisher-count checks.

#### 4. File-by-file

Shared files below are unchanged between `b76daa` and the #29 head, so the line numbers match both. CI, `.env`, and `sim.params` are the #19 versions; #29 rewrites those.

**`versions.env` (lines 4–19).** `PX4_VERSION=1.17.0`, `PX4_MSGS_REF=v1.17.0`, `XRCE_AGENT_VERSION=v2.4.3`, `ROS_DISTRO=jazzy`. CUDA arches `8.6;8.9` and `OPENCV_VERSION=4.11.0` apply only when `WITH_CUDA_OPENCV=1` (line 16, default 0). QGroundControl URLs on lines 18–19 are a `master` build and a `latest` AppImage. Those two URLs are not version pins. `scripts/check_versions.sh` compares Dockerfile `ARG` defaults to this file and requires `PX4_MSGS_REF` to equal `v${PX4_VERSION}`. It also requires the literal clone line `git clone --depth 1 -b v${PX4_VERSION}` for `px4_msgs`. `px4_ros_com` is cloned with `--depth 1` and no branch (`Dockerfile_px4_sim_NO_GPU` line 112). That clone still floats.

**`dockerFile/Dockerfile_px4_sim_NO_GPU`.** Lines 1–4 declare the three ARGs before `FROM ros:${ROS_DISTRO}-ros-base`, which is what `check_versions.sh` greps. Lines 93–95 clone PX4 at `v${PX4_VERSION}`. Lines 97–103 build Micro-XRCE-DDS-Agent at `XRCE_AGENT_VERSION`. Lines 115–118 copy `patch_dds_topics.py` into the image and run `make check_px4_sitl_default`, so the distance-sensor publication is compiled in. Lines 132–142 add Mesa llvmpipe and Xvfb in a later layer so a camera-rate change does not rebuild the firmware. The GPU Dockerfile starts `FROM nvidia/cuda:12.9.2-cudnn-devel-ubuntu24.04` and repeats the same version ARGs. The old GPU file’s typo `registeryGPU` and the broken image line (`px4GPUTAG:?error}"`) are gone.

**`includes/gz/patch_dds_topics.py` (lines 15–42).** Inserts a `/fmu/out/distance_sensor` publication immediately after the `publications:` header when that topic is missing from the publications section. A second run returns the text unchanged (line 32). The start script calls it again before `make` (see below), so a mounted PX4 tree and the image stay aligned. Risk: the insert is the first newline after `publications:`, not a YAML-aware edit. A comment between the key and the first topic still works; a `publications:` that is not at column 0 does not.

**`scripts/px4_params.py`.** `parse_params_text` (lines 40–50) accepts `NAME VALUE` and rejects anything else. `load_param_files` (lines 59–76) errors when two files set one name to two values. `env_overrides` (lines 79–90) lets `PX4_PARAM_*` replace file values; `PX4_PARAM_FILES` is the file list, not a parameter. `ensure_post_before_ekf2` (lines 127–149) finds the strings `ekf2 start` and `dataman start`, deletes the stock `.post` source if it sits after EKF2, and inserts it immediately before `dataman start`. `apply` (lines 195–211) writes that script beside every airframe whose name contains no dot (lines 179–188) and deletes `parameters.bson` plus `parameters_backup.bson` (lines 170–176). That clear is why a previous SITL boot cannot hide a new value. Risk: the hook is a substring search. A comment that contains `dataman start` before the real command moves the hook to the comment. `unknown_names` (lines 158–167) checks names against `parameters.xml` / JSON metadata, but only when you run `check-metadata`. Container start does not.

**`config/px4/params/sim.params` on this PR (4 parameters).** `NAV_DLL_ACT 0`, `NAV_RCL_ACT 1`, `COM_RC_IN_MODE 1`, `COM_RC_LOSS_T 35`. The comment in the later #29 file explains why `NAV_DLL_ACT` has to be a `.post` set and not `PX4_PARAM_NAV_DLL_ACT` before the airframe: `4001_gz_x500` `set-default`s it to 2, and the compiled default 0 does not mark the parameter configured. This PR does not yet set the five EKF2 values. Those arrive in #29.

**`includes/gz/startFiles/gz_start_px4_gz_sim.sh` (this branch, 81 lines).** Line 13 defaults `PX4_GZ_MODEL` to `x500_depth`. Line 17 defaults `PX4_GZ_MODEL_POSE` to `-3,-1.6,0,0,0,3.14` (z = 0). Lines 23–60 start Xvfb or prepend a `gz` wrapper that adds `--headless-rendering` when `GZ_HEADLESS_RENDERING=1`. Lines 63–73 patch DDS topics, `make px4_sitl_default`, then `px4_params.py apply`. Lines 77–81 export `PX4_GZ_WORLD` and exec `../bin/px4`. Custom worlds no longer need a cmake-time `gz_<model>_<world>` target. This is the right PX4 1.17 launch. It conflicts with #21, which still runs `make px4_sitl "gz_${MODEL}"` and hardcodes `MODEL=x500_depth`.

**`compose.yml`.** Lines 1–3 say the local GUI path is `docker-compose-px4.yml` and that file includes `compose.gui.yml`. `docker-compose-px4.yml` is only:

```yaml
include:
  - path: compose.yml
```

`compose.gui.yml` exists and only forwards `DISPLAY` / `XAUTHORITY`. `scripts/up.sh` never adds it. Local `./scripts/up.sh` therefore does not apply the GUI overlay the comment describes. The base file already interpolates `DISPLAY` (line 52), so a set `DISPLAY` still reaches the container. The overlay is redundant for that variable and the comment is wrong.

Lines 19–51 are the shared environment: `RTABMAPVIZ` defaults to `false` (line 23), `VISION_PROFILE` defaults to `cpu` (line 33), mount `CAM_*` defaults match the x500_depth include (lines 28–31), spawn pose z is 0 (line 51). The GPU overlay (`compose.gpu.yml` lines 8–22 and `docker-compose-px4-GPU.yml` lines 6–23) repeats those keys and sets `VISION_PROFILE` default `full`, then reserves NVIDIA GPUs with `deploy.resources.reservations.devices`. That is the Compose v2 GPU form. It replaces the old static IP network (`10.20.10.2`) and `container_name`. Scripts use `compose exec` because container names are gone.

Line 74 sets `privileged: true`. Line 58 bind-mounts `/dev`. `compose.ci.yml` lines 12–22 replace the PX4 volume list and drop `/dev` and the X11 socket. CI does not publish host ports (`ports: !override []`, line 12). QGroundControl and Nav2 RViz move to profile `ci-disabled` (lines 57–60). `privileged` stays.

The PX4 healthcheck (lines 77–84) runs `healthcheck.py --service PX4` after sourcing ROS and the workspace. `start_period: 180s` (line 79) covers the in-container `make`. `StatePublisher` waits on `service_healthy` (lines 116–118).

**`HealthCheck/healthcheck.py`.** `choose_topic` (lines 71–87) ranks candidates by “has a publisher”, then `_vN`, then list order. A subscriber that advertises `/fmu/out/vehicle_status` with zero publishers loses to `vehicle_status_v1`. `effective_min_rate` (lines 90–97) lets `HEALTH_CAMERA_MIN_HZ` lower only the `camera` group, which is how the llvmpipe flight avoided the 5 Hz floor. `judge_rate` (lines 100–108) skips absent topics only when `skip_if_absent` or `optional` is set. `HealthCheck/checks/px4.yaml` marks `/ground_truth/odom` `skip_if_absent: true` and `min_rate_hz: 45`, so this branch stays green without #21. Camera topics are gated with `when_env: CameraType: rgbd|stereo`.

**`scripts/smoke_test.sh` (lines 16–47).** Forces `HEADLESS=1` and `RTABMAPVIZ=false`, brings the stack up with `--wait`, runs `assert_px4_params.sh`, then healthchecks the camera group and the full PX4 graph. On this PR the CI flight does not use this script as the only gate. `scripts/run_e2e.sh` (+368) and `tests/e2e/*` fly the out-and-back and grade settle, overshoot, and yaw. The grading limits live in `.env` (`E2E_LEG_LENGTH_M=0.3`, settle ±2 cm for 1 s, yaw settle ±3°). #29 deletes the committed `.env` and stops calling `run_e2e.sh` from CI. The driver and the tests remain in the tree.

**`.github/workflows/` on this PR.** `ci.yml` calls reusable `_build.yml` and `_sim-test.yml`. Pull requests build the CPU image, upload it, and a second job loads it. A later commit builds the GPU image on pull requests and then stops doing that. The camera flight is `continue-on-error` and mentions PR #20. `nightly.yml` and two composite actions (`setup-buildx-login`, `collect-artifacts`) exist only on this head. #29 inlines the build and deletes those files.

**`includes/gz/tune_gz_cameras.py` (lines 25–32).** Rewrites `<update_rate>` inside `<sensor>` blocks whose type is `camera` or `depth_camera`. `gz_modifications.bash` calls it only when `GZ_CAMERA_UPDATE_RATE` is set, and only on the copy under the PX4 tree. Width and intrinsics stay. The regex is not an XML parser. A sensor element that nests another sensor, or a rate tag written across lines differently from `\s*`, is missed. The function returns exit 1 when nothing changes, which fails the container start if the variable is set and the model has no matching sensor.

#### 5. Verdict

Strengths. One Compose definition removes the GPU file’s YAML syntax errors and the duplicated service bodies. Parameter application is ordered relative to EKF2 and the SITL store is cleared, which matches how PX4 1.17 posix `rcS` actually behaves. Version drift between Dockerfiles and `versions.env` fails lint. Topic selection understands `_v1` names. The ground-truth health row does not require #21.

Risks. The GUI comment and `docker-compose-px4.yml` disagree. `px4_ros_com` and both QGC URLs are unpinned. `privileged: true` remains in CI. The `.post` insertion is a text search. The out-and-back gate on the old `feat/` workflow failed height, hover, settle, and yaw (the failure #29 cites as run 37896524085). This head still contains that driver.

Merge readiness. Do not merge this head. Use #29, which is the same work with the failing flight removed and the lint fixes applied. Keep the parameter hook, the DDS patch, and `compose.yml`.

#### 6. Follow-ups

- Pin `px4_ros_com` to a commit. Point QGC at a release asset, or drop QGC from the image used by CI.
- Make `docker-compose-px4.yml` include `compose.gui.yml`, or delete the overlay and fix the comment in `compose.yml` lines 1–3.
- Run `px4_params.py check-metadata` in the image job so a renamed EKF2 parameter fails before the flight.
- Replace the `dataman start` substring search with a match on a command line.

---

### PR #29 — Fix CI lint so the headless hover can run

Open draft. https://github.com/ParsaKhaledi/px4_sim_ros2/pull/29

#### 1. Summary

This is #19 retargeted at `main`, plus two commits. `3f364c76` collapses CI to one workflow and one headless smoke. `50cda67b` fixes the two pytest failures that blocked that workflow: the camera-env fixture still expected `RTABMAPVIZ=true`, and the `param show` sample omitted the five EKF2 rows now in `sim.params`. Lint, colcon, image build, and the job named `flight` are green on runs 37911324521 and 37911350600. The flight job runs `scripts/smoke_test.sh`. It does not arm, take off, or hover.

#### 2. Stack position

- Base: `main` (`5eb5fde`).
- Head: `feature/devops-compose-health-ci` at `50cda67b09ba922064a36a3f1fb42047d61822bf`.
- 28 commits. The first 26 match #19. Draft, open, not merged.
- Delta from #19 head `b76daa`: 18 files, +441 / −826.

#### 3. Stats

Against `main`: 77 files, +6696 / −1077.

Against #19, the deletions are `.github/workflows/_build.yml` (−442), `_sim-test.yml`, `nightly.yml`, both composite actions, and the committed `.env` (−53). `ci.yml` grows by the inlined build and the smoke step.

#### 4. File-by-file

**`.github/workflows/ci.yml`.** Line 27 adds `feature/**` to `push` branches, so a feature-branch push runs the workflow. The old workflow only ran from a pull request, which is why the closed `feat/` branch never reported a push result. Lines 79–85 copy `.env.example` to `.env` when `.env` is absent, then hadolint only `Dockerfile_px4_sim_NO_GPU`. The GPU Dockerfile is no longer linted. `check_versions.sh` still reads it. Lines 148–361 inline the old `_build.yml`: Buildx `v0.38.0`, `crazy-max/ghaction-github-runtime@v4`, cache `type=gha` with `version=2`. `image_rebuild_paths.py` can plan `pull` instead of `build` when the diff does not touch image inputs. Push to Docker Hub stays limited to `main` and tags `v*`.

Lines 358–428 are the flight job. It downloads the artifact when `image_mode=built`, loads it with `zstd`, and runs:

```yaml
CameraType: none
PX4_GZ_MODEL: x500
COMPOSE_FILES: docker-compose-px4.yml:compose.ci.yml
COMPOSE_SERVICES: PX4
GZ_HEADLESS_RENDERING: "0"
./scripts/smoke_test.sh
```

`timeout-minutes: 15` on that step (line 412) and `SMOKE_TEST_TIMEOUT: "240"`. There is no `run_e2e.sh`, no `px4_offboard.py`, and no `continue-on-error`. `tests/test_flight_workflow.py` (the whole file, 22 lines) locks those strings in. The step title is “Headless hover/smoke”. The script it calls never sends an offboard setpoint.

**`scripts/smoke_test.sh` lines 38–46.** After `assert_px4_params.sh`, it runs `healthcheck.py --group camera` and then the full PX4 check. With `CameraType=none`, every camera row in `HealthCheck/checks/px4.yaml` fails `topic_matches_env` (`when_env` is `rgbd` or `stereo`). Process rows have no `group`, so `selected()` also drops them when `--group camera`. `HealthRunner.run` (lines 244–253) still enters `_ros_checks` because `self.config.get("topics")` is true even when the filter selects nothing. The camera invocation therefore exits 0 with `ok=0 fail=0` after a ROS init. It does not prove a camera. The second invocation does prove `/clock` at 50 Hz, versioned odometry at 20 Hz, and `vehicle_status_v1`. `/ground_truth/odom` stays optional.

**`config/px4/params/sim.params` lines 9–13.** Adds `EKF2_GPS_CTRL 0`, `EKF2_EV_CTRL 9`, `EKF2_HGT_REF 0` (baro), `EKF2_MAG_TYPE 5`, `EKF2_RNG_CTRL 1`. `docs/plans/devops.md` says these five are owned together with `feature/px4-control`, and that the magnetometer plugin stays on the sim branch. On a `CameraType=none` smoke, GPS fusion is off and external vision is on, and nothing publishes vision. The smoke only checks that `param show` matches. It does not check estimator flags. A vehicle can pass this job with an estimator that will not fly.

**`tests/test_px4_params.py`.** The sample dump now includes the five EKF2 lines (the block added after `COM_RC_LOSS_T`). `test_ekf2_mismatch_still_fails` rewrites `EKF2_EV_CTRL` to 0 and expects `EKF2_EV_CTRL is 0`, and expects no “missing from param show”. `test_repo_sim_file_parses` asserts the five values. That is the lint fix. It does not execute PX4.

**`tests/test_compose_config.py` around the `CAMERA_ENV` dict.** `RTABMAPVIZ` expected value changes from `"true"` to `"false"`, with a comment that an unset viewer stays closed. CPU and GPU service environments must still match. `scripts/up.sh` drops the `GPU=1` branch and always uses `docker-compose-px4.yml`. `compose.gpu.yml` and `docker-compose-px4-GPU.yml` remain in the tree. CI no longer composes them (`ci.yml` yamllint list omits `compose.gpu.yml` and the GPU entrypoint). A GPU syntax regression is invisible to this workflow.

**`.env`.** Deleted. `.gitignore` ignores it. `.env.example` keeps the image tags, the spawn pose with z = 0, and the E2E numeric limits that CI no longer reads. `docs/plans/devops.md` states the cut: no nightly, no composite actions, no second image in this plan, no offboard controller in CI. The plan’s “Done when” says the workflow is green and the flight is one headless hover/smoke. The workflow is green. The flight is a boot-and-rate smoke.

#### 5. Verdict

Strengths. The two lint failures match the code that is actually committed. Feature-branch pushes run the same jobs as pull requests. The image job loads one artifact instead of building twice. `.env` is no longer a committed secret-adjacent local file. Parameter readback includes the EKF2 set and still fails a wrong `NAV_DLL_ACT` (`assert_px4_params.sh` also re-runs the checker with `NAV_DLL_ACT=99` and requires a non-zero exit).

Risks. The job name says hover. The process is “PX4 reaches healthy topics and the parameter file applied.” `EKF2_GPS_CTRL 0` plus `EKF2_EV_CTRL 9` on a camera-less model is the indoor-vision set with no vision source. GPU compose files are leftover and unlinted. `privileged` remains. The 15-minute step budget includes an in-container `make px4_sitl_default` on every smoke; a cold ccache can time out without saying the estimator failed.

Merge readiness. Acceptable as the DevOps base once you rename the step to “headless boot smoke” or you add a real hover. Do not treat the green `flight` check as evidence the vehicle hovered. Merge this before #21 and #20 so those branches rebase onto one Compose file. It is still a draft; mark it ready when the step name matches the script.

#### 6. Follow-ups

- Either call a one-waypoint hover and assert `vehicle_land_detected` plus a height band, or rename the step and the plan’s “hover” wording.
- Split `sim.params`: failsafe parameters for the camera-less smoke, EKF2 vision values only when the vision stack is the service under test.
- Delete or lint `compose.gpu.yml` and `docker-compose-px4-GPU.yml`. The plan says the second image is out of this work. The files are still in the diff.
- Skip `healthcheck.py --group camera` when `CameraType=none`, so the log does not show an empty pass.

---

### PR #21 — Headless Gazebo, ground truth, and trajectory evaluation

Closed, not merged. https://github.com/ParsaKhaledi/px4_sim_ros2/pull/21

#### 1. Summary

This branch makes a headless Gazebo server render sensors, publishes a ground-level `world` → `spawn` TF, and adds an odometry plugin that is bridged as `/ground_truth/odom` without feeding PX4’s external-vision topic. It also slices world collisions into `includes/gz/walls/<world>.json`, vendors Fuel models, and adds `sim_monitor` plus `trajectory_eval`. The line count is mostly meshes and generated wall JSON. The control behavior you need from this PR is the spawn frame, the ground-truth topic, and the JSON wall schema that #28 loads.

#### 2. Stack position

- Base ref name: `dev/px4-upgrade`. Parent of `a33f79d4`: `5eb5fde` (`origin/main`).
- Head: `feat/sim-headless-groundtruth` / `origin/feature/sim-headless-groundtruth` at `2abc7f9f8cee4a8d00e0d95c0041f6ed39941a83`.
- 21 commits. Closed 2026-10-09. Not merged.
- Sibling of #19 and #20. Both this PR and #19 replace `gz_start_px4_gz_sim.sh` and `docker-compose-px4.yml`.

#### 3. Stats

191 files, +274726 / −175.

Of that, 94 mesh/texture files are about +216414 lines. The remaining 97 files are about +58312 / −175, and most of those lines are generated walls: `includes/gz/walls/apt_world.json` (+21841), `sonoma_raceway.json` (+15409), `husarion_office.json` (+13633). Hand-written logic is on the order of a few thousand lines. `wall_geometry.py` is 604. `checks.py` is 509. `preflight_check.py` is 471.

#### 4. File-by-file

**`includes/gz/bin/gz` (lines 22–40).** If the first argument is `sim`, `HEADLESS=1`, and the backend is not `xvfb`, the wrapper scans for `-s`. When the launch is a server and `--headless-rendering` is absent, it execs `REAL_GZ sim --headless-rendering …`. GUI launches and `--versions` queries pass through. Lines 17–19 refuse to run if `REAL_GZ` resolves to the wrapper, which avoids a PATH loop. #19’s start script writes a different wrapper under `/tmp/gz-bin/gz` when `GZ_HEADLESS_RENDERING=1`. After a merge, one wrapper has to win. This one keys off `HEADLESS`. The DevOps one keys off `GZ_HEADLESS_RENDERING`.

**`includes/gz/startFiles/gz_start_px4_gz_sim.sh`.** Line 21 hardcodes `MODEL="x500_depth"`. It does not read `PX4_GZ_MODEL`, so #29’s `PX4_GZ_MODEL=x500` smoke cannot select the plain airframe on this script. Line 24 defaults the pose to `-3,-1.6,0.15,0,0,3.14`. z is 0.15 m of drop clearance, not a ground height. #19/#29 default z to 0. #28’s spawn TF uses z = 0 either way; the vehicle pose does not. Lines 32–39 run `sim_origin.py` and `eval` its shell exports. A hostile string in the world file could be evaluated; the script only evals that tool’s output, which is the right shape if `shell_exports` stays a fixed set of assignments. Lines 41–47 patch ground truth and the downward lidar with warnings on failure, and exit 1 when `patch_x500_sensor_systems.py` cannot find the world. Lines 63–87 select Xvfb or EGL and optionally `LIBGL_ALWAYS_SOFTWARE`. Lines 96–99 export `PX4_GZ_WORLD` only when `WORLD` is not `default`, then `make px4_sitl "gz_${MODEL}"`. #19 switched to `make px4_sitl_default` plus `PX4_GZ_WORLD` for every world, including default, because `gz_<model>_<world>` targets exist only for worlds present at cmake time. This script regresses that for a custom world whose ninja target was never generated, and it ignores the DevOps parameter hook entirely. There is no `px4_params.py apply` here.

**`includes/gz/scripts/sim_origin.py`.** Lines 32–35 default the spawn geodetic point to latitude 35.7048378, longitude 51.4095049, altitude 1205 m (Amirkabir), and the same pose string as the start script. `world_origin_from_spawn` (line 99) shifts that point by the spawn east/north so PX4’s `PX4_HOME_LAT/LON/ALT` is the world origin, which is what PX4 v1.17 Gazebo uses. The vehicle’s first GPS fix stays at the spawn. `rewrite_spherical_coordinates` (line 133) rewrites `<spherical_coordinates>` in the copied worlds. Compose on this branch passes `SIM_ORIGIN_LAT/LON/ALT` (`docker-compose-px4.yml` lines 58–61). The DevOps Compose file does not have those keys. A rebase has to add them to `compose.yml`’s `x-ros-env` or the origin silently falls back to the Python defaults.

**`includes/gz/sim_ws/src/sim_monitor/sim_monitor/spawn_frame.py` (lines 22–29).** `spawn_ground_pose` copies x, y, and yaw and sets z, roll, and pitch to 0. The static TF `world` → `spawn` sits on the floor under the drop pose. Ground truth is broadcast as `world` → `base_link_gt` (`GT_CHILD_FRAME`), not `base_link`, because RTAB-Map already parents `base_link` from `odom`. #28 looks up exactly `world` → `spawn` and ignores z. The two branches agree on that frame name. They do not agree on the default pose string (0.15 versus 0). Yaw `3.14` is in both defaults.

**`includes/gz/scripts/patch_x500_ground_truth.py`.** Inserts `gz::sim::systems::OdometryPublisher` with `/ground_truth/odom`, `/ground_truth/odom_with_covariance`, and TF `/ground_truth/tf`. `includes/gz/config_gz_bridge_sim.yaml` bridges only `/ground_truth/odom` as `nav_msgs/Odometry`. The covariance topic is intentionally not the PX4 vision topic.

**`includes/gz/sim_ws/src/sim_monitor/sim_monitor/checks.py` lines 334–366.** `ground_truth_leak_from_info` FAILs when `gz topic -i` shows a publisher on `/model/<model>/odometry_with_covariance` (default model `x500_depth_0`, lines 276–279). That topic is what GZBridge feeds EKF2 as external vision. No publisher is PASS. A missing `gz` binary is SKIP (return true). A non-zero `gz topic` status with no publisher list is FAIL, including a timeout in the caller (`preflight_check.py` lines 165–168). Risk: SKIP when the CLI is absent is a pass. A container that dropped `gz` from `PATH` because the wrapper was wrong will not catch a leak.

**`preflight_check.py` `PoseOrigins` (lines 98–133).** The ground-truth origin is captured when the RTAB-Map origin is captured, not on the first ground-truth sample. The comment states the vehicle spawns 0.15 m up and drops before RTAB-Map publishes. A lost tracker or a jump back to the identity pose recaptures both. `jumped_to_identity` uses 0.05 m and 0.50 m thresholds (module constants `_IDENTITY_M`, `_JUMP_M`). If RTAB-Map never publishes, the pose check has no origin. The service can still report other gates.

**`checks.py` rates (lines 232–268).** Camera minimum is half the expected rate. Expected camera rate is `CAM_RATE_HZ`, else 10 Hz on `VISION_PROFILE=cpu`, else 30 Hz. IMU minimum is `IMU_MIN_FRACTION` (0.75) times the resolved rate. `default_min_rtf` is 0.15 when `HEADLESS_SOFTWARE` is set, otherwise 0.8, unless `PREFLIGHT_MIN_RTF` is set. `RTF_CPU_HINT` (lines 375–377) points at `VISION_PROFILE=cpu` and cites the same ~0.45 versus ~0.125 measurement as #20. This branch imports `geometry` optionally (`_load_geometry`, line 152) so it still runs before #20 is merged. A missing module falls back to the SDF and then to a constant.

**`includes/gz/startFiles/gz_start_sim_helpers.sh` (lines 29–43).** Backgrounds real-time factor, spawn frame, preflight, and `px4_imu_relay` only when `IMU_SOURCE=px4`. The default is `oak`. `px4_imu_relay.py` documents `IMU_STAMP_MODE=receive`: the relay stamps with the ROS clock at receive time because a PX4 boot timestamp fails RTAB-Map’s `wait_imu_to_init` comparison against image stamps. Receive-time stamps add the bridge latency into the IMU stream. That is the right tradeoff for the sync gate and the wrong timestamp if you later fuse this IMU in EKF2 with a tight delay parameter.

**`includes/gz/scripts/wall_geometry.py`.** Slices box, cylinder, and mesh collisions at `slice_z` (default 0.5 m in `export_world`, line 560) into segments with `start`/`end`, `source`, and `kind` in `box|cylinder|mesh`. `load_collada` (line 316) is the fragile parser. Generated JSON for three worlds is committed. #28’s loader requires `frame` of `world_enu` or `local`, `units` of `m`, and `segment_count` equal to `len(segments)` when the field is present. Those documents match. Risk: regenerating walls after a mesh edit is a 50k-line diff, and a hand edit to a JSON will not match the mesh. The README under `includes/gz/walls/` is the contract; treat the JSON as build output that you vendor on purpose.

**`includes/gz/models/FUEL_NOTICE.md`.** Furniture and Sonoma Raceway are vendored from Fuel. The notice attributes CC BY 4.0 models to Open Robotics and lists CC0 models. Do not drop this file in a later cleanup. Two `__pycache__/*.pyc` files are added (`includes/gz/x500_tf_publisher/` and `includes/gazebo_classic/...`). Those are bytecode, not sources. Delete them before any merge.

**`docker-compose-px4.yml` lines 30–61.** This is still the old Compose shape: `container_name`, static IP `10.20.10.2`, healthcheck calling `check_multi_topic_pub.bash` on `/clock` and `/fmu/out/vehicle_odometry` (unversioned). #19 deletes that bash script. This healthcheck fails as soon as the DevOps branch is the base. The new keys you want to keep are `HEADLESS`, `HEADLESS_BACKEND`, `HEADLESS_SOFTWARE`, `IMU_SOURCE`, `LIDAR_DOWN`, `PREFLIGHT_*`, `SIM_ORIGIN_*`, and `PX4_GZ_MODEL_POSE` without forcing z = 0.

**`trajectory_eval/metrics.py`.** `absolute_trajectory_error` (lines 93–114) reports an unaligned error relative to the first sample and, when there are at least three poses, an SE(3) alignment with `"scale": 1.0`. It does not estimate scale. `relative_pose_error` (lines 123–167) walks ground-truth path length, accepts a segment within 25% of `distance_m`, and defines `drift_m_per_m` as translation RMSE divided by the nominal segment length, not by the measured segment. A segment that is 1.24× the nominal length still divides by the nominal. That matches a fixed-delta drift number and misstates drift if you expected “per metre actually flown.” `frames.py` lines 96–106 convert NED/FRD to ENU/FLU with a fixed quaternion (`quat_from_rpy(pi, 0, pi/2)` and a 180° aircraft-to-base rotation). GPS samples use a separate geodetic path (`geodetic_to_enu`, line 144). Mixing a GPS ENU trace with a Gazebo ENU trace without the origin from `sim_origin.py` produces a large ATE that is a frame error.

**Worlds.** `husarion_office.sdf` is +147 / −145. `apt_world.sdf`, `empty_with_plugins.sdf`, `husarion_world.sdf`, and `sonoma_raceway.sdf` are added. The start script copies repo worlds into the PX4 tree via the sensor-system patch and `GZ_SIM_RESOURCE_PATH` (line 50). Offline resolution is checked by `check_offline_worlds.py` and `fuel_assets.py`.

#### 5. Verdict

Strengths. Headless rendering is a small, testable wrapper with a REAL_GZ loop guard. The spawn frame separates drop height from the ground frame #28 needs. Ground truth is a different topic from PX4 vision, and a preflight check fails when those are the same. Wall JSON is a documented schema with a geometry generator and tests. Trajectory metrics refuse scale alignment. Fuel licensing is written down. The monitor degrades when the OAK-D geometry module is absent, so this branch can land before #20.

Risks. `make px4_sitl "gz_x500_depth"` and a hardcoded model fight #29. z = 0.15 versus z = 0 changes where the vehicle is born. The old Compose healthcheck calls a script #19 deletes. Vendored meshes dominate review and clone time. Bytecode is committed. Ground-truth leak SKIP-on-missing-CLI can hide a bad PATH. Preflight pose error depends on RTAB-Map having published.

Merge readiness. Not mergeable on top of current `main` or on top of #29 without a rebase that keeps `make px4_sitl_default`, `px4_params.py apply`, versioned healthchecks, and the new environment keys. Land the Python packages, the `gz` wrapper, `sim_origin.py`, the patches, and the wall generator. Do not take this Compose file as the base.

#### 6. Follow-ups

- Rebase onto #29. Put `HEADLESS*`, `SIM_ORIGIN_*`, `IMU_SOURCE`, `LIDAR_DOWN`, and `PREFLIGHT_*` into `compose.yml`. Delete `container_name` and the bash healthcheck.
- Honor `PX4_GZ_MODEL`. Default pose z belongs in one file. #29’s `-3,-1.6,0` and this branch’s `…,0.15,…` cannot both be the default.
- Stop committing `__pycache__`. Generate wall JSON in CI or keep it and add a check that `wall_geometry.py` reproduces the committed documents.
- Make a missing `gz` CLI a FAIL for the vision-leak check inside the container, and a SKIP only in unit tests.

---

### PR #20 — OAK-D S2 model and stereo / RTAB-Map

Closed, not merged. https://github.com/ParsaKhaledi/px4_sim_ros2/pull/20

#### 1. Summary

One Python module, `includes/gz/oakd_s2/geometry.py`, owns the S2 housing, the 7.5 cm baseline, the left-camera calibration, and the mount. Container start renders both SDFs and the URDF from it, then patches the `x500_depth` include. Stereo RTAB-Map uses exact sync. A relay republishes the right `camera_info` with `P[3] = -fx * 0.075` and the same stamp. `VISION_PROFILE=cpu` quarters the pixels so llvmpipe can hold the camera rate in sim time. The Gazebo folder name stays `OakD-Lite` so PX4’s `model://OakD-Lite` include still resolves. The link name is `camera_link`.

#### 2. Stack position

- Base ref name: `dev/px4-upgrade`. Parent of `af02e3af`: `5eb5fde` (`origin/main`).
- Head: `feat/oakd-s2-vision` / `origin/feature/oakd-s2-vision` at `8a27573edeac5616f53f8aad14381f65735e3d77`.
- 6 commits. Closed 2026-10-09. Not merged.
- The PR body says to rebase onto #19. GPU pass-through of `CAM_*` and `VISION_PROFILE` is implemented on #19/#29 (`compose.gpu.yml` lines 8–22), not in this diff. This diff edits the old `docker-compose-px4.yml` (+35).

#### 3. Stats

30 files, +4159 / −294.

Largest: `includes/gz/oakd_s2/test_oakd_s2.py` (+820), `HealthCheck/rtabmap_health_log.py` (+585), `render_oakd.py` (+516), `geometry.py` (+502). SDF rewrites are +156/−100 and +164/−102. No mesh vendor dump.

#### 4. File-by-file

**`includes/gz/oakd_s2/geometry.py`.** Line 30 sets `BASELINE_M = 0.075`. It is not an environment variable. Lines 37–43 set the mount default to `0.12, 0.03, 0.242` and pitch `17` degrees, which is the old 0.3 rad sensor pitch moved onto the mount. Lines 51–55 store the left calibration at 640×400 (`fx` 406.6239, `fy` 406.0800). The right camera uses that same K. `stereo_tx` (lines 250–258) returns `-fx * baseline`. At full resolution that is about −30.50 pixels, which is the Tx the PR body cites. `scale_intrinsics` (lines 207–210) scales fx, fy, cx, cy with the resolution ratio so the field of view stays. `profile_from_env` (lines 181–204) accepts only `full` and `cpu`, then lets `CAM_STEREO_WIDTH`, `CAM_RATE_HZ`, and `IMU_RATE_HZ` override fields. A non-positive rate raises `ValueError`, and `gz_modifications.bash` runs the renderer with `|| exit 1`, so a bad override fails the container instead of rendering a zero-rate sensor. `FULL_PROFILE` is 640×400 stereo, 640×480 color, 30 Hz cameras, 200 Hz IMU. `CPU_PROFILE` (lines 160–168) is 320×200, 320×240, 10 Hz, 100 Hz.

Distortion `D` is zero. The renderer is a pinhole. Real k1/k2 from the calibration YAML are not applied. That is consistent as long as every consumer uses the rendered `camera_info`. A node that loads the real YAML will disagree with the image.

**`render_oakd.py` `render_urdf` (lines 366–410).** The URDF comment is a sentence that ends before the pose numbers. An earlier revision put a colon in that comment; `launch_ros` then rejected `robot_description` and the state publisher crash-looped. The checked-in `x500_urdf.urdf` line 2 matches this generator: “Generated from includes/gz/oakd_s2. camera_joint matches …”. Pitch is on `camera_joint` (line 410), not on the optical joints. Optical children exist for left, right, and color. Depth reuses `camera_rgb_optical_frame`. `write_models` rewrites the SDF and URDF inside the mounted repo. The PR body says this leaves the checkout dirty. That is still true: a CI run that starts the container on a bind mount will show SDF diffs you did not author.

**`includes/gz/gz_modifications.bash`.** Renders before the stereo/rgbd folder swap (`render_oakd.py`, early in the script) and patches `x500_depth` plus `CameraJoint` when that model is present. A missing model logs and continues, so the URDF can still update. #19’s copy of this script adds the `none` / `flight` branch and `tune_gz_cameras.py`. This copy does not know `CameraType=none`. Rebase has to keep both the renderer and the flight-only early return. Running the renderer on a `CameraType=none` smoke would still dirty the tree and is unnecessary.

**`stereo_info_relay.py` (lines 35–42).** Overwrites `k`, `p`, `r`, and `d` on the right `CameraInfo` and publishes it. It does not copy into a new message first, but it does not change `header`, so the stamp stays the Gazebo stamp. `use_sim_time` is forced true (line 30). Topic names come from `geometry` (`RIGHT_INFO_IN` → `RIGHT_INFO_OUT`, the `_baseline` topic). `gz_start_rtabmap_stereo.sh` lines 37–41 point RTAB-Map at `right_camera_info_topic:=/camera/stereo/right/camera_info_baseline`, `approx_sync:=false`, `wait_imu_to_init:=true`, `use_sim_time:=true`, and `rtabmap_viz:=${RTABMAPVIZ}`. Exact sync is valid only while both cameras and this relay share a stamp. If the relay is down, the baseline topic has no publisher and exact sync never pairs. There is no approximate-sync fallback.

**`rtabmap_profile.sh` (lines 11–54).** `cpu` sets `Vis/MaxFeatures 400`, `Vis/MinInliers 15`, `Kp/MaxFeatures 300`, `Stereo/MaxDisparity 64` for stereo only, `Odom/VisKeyFrameThr 30`, `OdomF2M/MaxSize 1000`. `full` uses 1000 features, 20 inliers, and does not set `VisKeyFrameThr` (RTAB-Map default 150). Both set `Odom/ResetCountdown 1`, `Odom/Strategy 0` (F2M), feature type 10 (ORB-OCTREE), and `Vis/EstimationType 1` (PnP). The comment on lines 48–49 says keyframe threshold 30 is not measured live. `Grid/NormalsSegmentation` is replaced by a real `Grid/` key. Unknown keys are dropped by RTAB-Map with no warning, which is why the script comments each name. RGB-D stays on approximate sync in the other start script. Grid heights on the wrapper move to 0.5 / 2.2 m.

**`includes/gz/config_gz_bridge.yaml` lines 52–58.** Bridges absolute `/imu` as `sensor_msgs/Imu`. The SDF topic is absolute so Gazebo does not prefix the model name. RTAB-Map’s `imu_topic:=/imu` matches. #21 will also bridge `/imu` when `IMU_SOURCE=oak` from a separate YAML, and its comment says not to bridge `/imu` from this file as well. A merged bridge list that includes both publishes the same ROS topic twice.

**`HealthCheck/rtabmap_health_log.py`.** Defaults: median features 120 (cpu 40), inliers 20 (cpu 15), odometry 7 Hz in sim time (lines 58–62, 134–147). `inlier_gate_value` (lines 223–236) drops lost frames and drops a 0-inlier frame that immediately follows a lost frame, which is the `Odom/ResetCountdown 1` restart. `--min-inliers` overrides only that gate. Wall-clock rate is reported and not gated, so a slow machine fails only when sim-time Hz drops. `scripts/eval_slam_accuracy.py` lines 28–38 call `evo_ape` and `evo_rpe` with `-a` only. `BANNED_ALIGNMENT` rejects `-as`, `-s`, and `--correct_scale`. Thresholds default to 0.50 m APE RMSE and 0.10 m/m drift. Exit 2 when evo is missing. Nothing in CI calls this script. The floors are starting gates. The PR body says `Odom/VisKeyFrameThr 30` was added after the measured runs.

**`x500_urdf.urdf`.** `camera_link` mass 0.091 kg, box `0.0229 0.097 0.0295` (depth × width × height, metres). Optical joints are children of `camera_link`. The old URDF used ±0.05 m stereo y and a joint rpy of about −1.85 rad. Those values are gone. `check_urdf` is cited in the PR body as passing; this review did not re-run it.

**Tests.** `test_oakd_s2.py` (+820) covers Tx, FOV, shared K, profile scaling, exact-sync flags, the reset-frame inlier rule, and the URDF comment loading as a YAML string. That is the right place for the colon regression.

#### 5. Verdict

Strengths. Geometry has one source, and the tests lock the numbers that used to drift between SDF, URDF, and `camera_info`. Exact sync matches how Gazebo stamps a stereo pair. The CPU profile is derived (scaled intrinsics, disparity cap halved with fx) and tied to a written measurement. The health log scores sim time, which is what EKF2 sees. The URDF comment crash is fixed at the generator, not only in the checked-in file.

Risks. Rendering into the bind-mounted repo dirties git. Exact sync has a hard dependency on the relay. IMU noise is a BMI270 stand-in, not a BNO086 specification. `VisKeyFrameThr 30` is unmeasured. This Compose edit conflicts with both #19 and #21. Dual `/imu` bridges appear if you concatenate the YAML files. Stereo and RGB-D remain mutually exclusive because `CameraType` still swaps one model directory onto `OakD-Lite`.

Merge readiness. Rebase onto #29 after #21, or onto #29 alone if you can defer walls. Do not merge the `docker-compose-px4.yml` hunk. Move `CAM_*`, `VISION_PROFILE`, and `RTABMAPVIZ` solely through #29’s `x-ros-env`. Keep `geometry.py`, the renderer, the relay, the profile script, and the tests.

#### 6. Follow-ups

- Render into a temp directory or into the PX4 model tree only. Leave `includes/gz/models/OakD-Lite-*` and the URDF as committed outputs of `full`, and fail CI if `VISION_PROFILE=full` would change them.
- Start the relay in the same script that sets `approx_sync:=false`, and fail health when `/camera/stereo/right/camera_info_baseline` has no publisher.
- Measure `Odom/VisKeyFrameThr 30` on the CPU profile before treating 7 Hz as the gate for motion, not only hover-at-rest.
- When merging with #21, bridge `/imu` from exactly one YAML, selected by `IMU_SOURCE`.

---

### PR #28 — Align geofence walls to the spawn TF and EKF2

Closed, not merged. https://github.com/ParsaKhaledi/px4_sim_ros2/pull/28

#### 1. Summary

The geofence stops reading `x1 y1 x2 y2` text files. It reads `includes/gz/walls/<world>.json`. A `local` map is used as written. A `world_enu` map stays inactive until the static TF `world` → `spawn` can be matched to a landed `vehicle_local_position` with EKF2 `cs_yaw_align` and `xy_valid`. The rotation is local NED heading minus the spawn heading after the ENU/NED conversion in `frames.py`. There is no GPS-versus-vision branch. Arming freezes the frame. A `heading_reset_counter` change re-derives it on the ground and, in flight, brakes to a hold and clears the geofence. GitHub reports the PR `CONFLICTING` / `DIRTY` against `feat/px4-control`.

#### 2. Stack position

- Base: `feat/px4-control` at `7e7887ff16121621ec7fe1d3bd8545445892ab11`.
- Head: `cursor/geofence-json-walls-83ac` at `cdbfcf315b64c4fa08117f16d8e13ba67b3d9a8f`.
- One commit. Closed 2026-10-09. Not merged. The branch ref is not on `origin` (fetch failed). The diff is still available from the pull request.
- Depends on #21 for real wall files and for the `spawn` frame. The node does not parse `PX4_GZ_MODEL_POSE`.

#### 3. Stats

12 files, +719 / −58.

`geofence.py` +290 / −18, `test_geofence.py` +209 / −9, `node.py` +124 / −14, `frames.py` +44. The text example is deleted. `walls_format_example.json` is +15.

#### 4. File-by-file

**`includes/gz/startFiles/gz_start_px4_control.sh` (the one-line path change).** `PX4_WALL_SEGMENTS_FILE` now points at `includes/gz/walls/${WORLD_NAME}.json` instead of `includes/gz/worlds/walls/${WORLD_NAME}.txt`. The export still happens only when the file exists. A world with no JSON leaves the parameter empty, and the node treats that as an empty wall set.

**`config/walls_format_example.json`.** `frame: local`, `units: m`, `segment_count: 2`, two segments with `start`/`end`/`source`/`kind`. The description says the file is an example and is not loaded. `px4_control.yaml` comment changes to the same path. `package.xml` adds `tf2_ros`.

**`frames.py` `world_local_rotation` (new, starting at line 185).** `rotation = wrap_pi(local_heading_ned - enu_yaw_to_ned(world_yaw_enu))`. `enu_yaw_to_ned` is `pi/2 - yaw_enu` (existing function, line 182). GPS/mag, where local heading equals the world NED heading, yields a rotation near 0. Vision, where local heading is 0 at the spawn, yields the negated world heading. The same subtraction produces both. `world_enu_point_to_local` (lines 211–233) subtracts the spawn ENU position, rotates the offset by `-rotation`, and adds the vehicle’s local east/north. Local east is `vehicle_local_position.y` and local north is `.x`, which is NED. The function does not use z.

**`geofence.py` loader.** `load_wall_file` rejects a `.txt` path with “the text wall list is not a wall format”. JSON must be an object. `frame` must be `world_enu` or `local`. `units` must be `m`. `segment_count`, when present, must equal the list length. `kind`, when present, must be `box`, `cylinder`, or `mesh`. A `world_enu` file returns `needs_alignment=True` and `walls=()` so world coordinates are never used as local ones by mistake. A `local` file returns those segments as the active walls.

**`WallAlignment.update`.** If `heading_reset_counter` changes, `_clear_installed` drops the active segments and clears `frozen`. When `landed` is false the step returns immediately with `brake=True` and the log line “EKF2 heading reset in flight; braking to hold and turning the geofence off”. When landed, it logs the re-derive and continues, so a new match can install in the same call. Installation requires a finite spawn, `landed`, `yaw_aligned`, and a fix with `xy_valid` and finite x, y, and heading. Arming sets `frozen` and logs the rotation. A second update while frozen and aligned returns no new log. If the vehicle arms before a frame exists, one warning is logged (“armed before the wall frame was aligned”) and the walls stay empty. Missing TF logs once (“static TF world → spawn is not available”) and does not repeat.

**`node.py`.** A steady clock (`ClockType.STEADY_TIME`) is created next to the node (about line 111). Control `dt` stays on the node clock (`motion.py` only adds a comment: `dt` is the gap between ticks). `_spawn_from_tf` imports `tf2_ros` inside the function. `ImportError` returns `None`, which surfaces as the generic “TF not available” warning, including when the dependency is simply not installed. The lookup is `lookup_transform('world', 'spawn', rclpy.time.Time())`. Time zero is correct for a static transform. The `except Exception` around the lookup also swallows programming errors in the quaternion conversion. `TransformListener` is created on first use with `spin_thread=False`. The first timer tick usually misses the TF; the next ticks succeed. That matches the “log once, keep trying” loop.

`_update_wall_frame` runs at the start of `_on_timer`, copies `landed`, `armed`, and the estimator under `_lock`, then calls `WallAlignment.update` outside the lock. It writes `_walls` back under the lock and, on `brake`, calls `self._motion.abort(now, …)` while holding `_lock`. If `abort` ever takes that same lock, this deadlocks. Confirm `abort` only touches motion state.

Discovery waits move to the steady clock: preflight service (1 s), the preflight call (3 s), the wait for the first `vehicle_status` and `vehicle_odometry`, and the parameter read-back (up to 20 s). `_wait_until` remains on the node clock for mission timeouts. `drone.py` `_wait_discovery` uses `time.monotonic()` with a 15 s cap and `spin_once`. A stuck `/clock` no longer blocks “is the server advertised.” A long sim-time arm timeout still uses sim time, which is what you want for the in-flight limit and what you do not want for DDS discovery. The split matches that.

**`test_geofence.py`.** `test_spawn_yaw_90_deg_wall_stays_north_for_vision_and_gps` builds a wall 1 m north of spawn `(1, 2)` in world ENU, with spawn yaw `+pi/2` (NED heading 0). Vision heading 0 produces rotation 0 and a wall from `(0, 1)` to `(1, 1)` in local metres. A GPS fix with the same heading and a local offset moves the wall by that offset and keeps it north. Arming freezes it; a second armed update is silent. `test_heading_reset_rederives_while_landed_and_brakes_in_flight` uses spawn yaw 0. Vision heading 0 rotates a north wall to the west (`rotation ≈ -pi/2`, wall x = −5). A landed counter change to the GPS heading rotates it back to the north. A further counter change with `landed=False` and `armed=True` sets `brake` and clears `walls`. `test_missing_spawn_tf_leaves_the_geofence_off` checks the one-shot log. `test_world_frame_alias_is_rejected_and_text_is_not_a_format` rejects `frame: world` and a `.txt` file. `yaw_from_quaternion(0,0,√2/2,√2/2)` is `pi/2`.

The tests do not cover a heading reset that arrives while landed and already armed (the code re-derives and re-freezes in one call because `armed` is still true). They do not cover a non-finite spawn. They do not start a ROS node, so the TF lookup and the lock around `abort` are untested.

#### 5. Verdict

Strengths. The frame rule is one formula, and the tests show the two estimator behaviors falling out of it. World coordinates cannot leak into the speed limiter before alignment. A heading reset in flight removes the fence and asks for a hold instead of braking against a stale wall. Text files fail closed. Steady-clock discovery is separated from sim-time mission timeouts, which is the failure mode of a paused `/clock`. The path change matches #21’s writer.

Risks. The PR does not merge cleanly onto its base. `except Exception` hides a missing `tf2_ros` and a bad quaternion in the same warning. `abort` under `_lock` needs a check. Walls stay off if the spawn TF is late and the vehicle is armed from a script that does not wait; the node logs and continues with no fence. Yaw-only alignment ignores a spawn whose floor is not z = 0 in the JSON slice. #21 slices at 0.5 m and puts the TF at z = 0. That is consistent. A future spawn frame that puts z at the drop height (0.15 m) would still be ignored here, which is what you want, as long as the JSON stays a ground slice.

Merge readiness. Do not merge until it rebases cleanly onto `feat/px4-control` and #21’s wall files exist on that branch. The math and the tests are in good shape. The integration (TF in a live graph, abort under the node lock) is not what the unit tests run.

#### 6. Follow-ups

- Rebase onto the current control branch. Resolve the conflict in `node.py` and `geofence.py` by keeping the steady-clock waits and `WallAlignment`.
- Log `ImportError` for `tf2_ros` as its own message. Narrow the lookup exception to `tf2_ros.LookupException` and connectivity errors.
- Call `abort` without holding `_lock`, or document that `abort` does not acquire it.
- Add the landed-and-armed heading-reset case to `test_geofence.py`.
- Point CI at one committed `includes/gz/walls/*.json` from #21 so this loader fails if the schema drifts.

---

### Cross-PR merge order

#29 is the only open PR, and it is the base the others have to sit on. #21 and #20 both fork from the same `main` commit and both rewrite Compose and `gz_start_px4_gz_sim.sh`. Rebase #21 first (spawn, ground truth, walls, headless `gz` wrapper, origin), then #20 (renderer and RTAB-Map) onto that, then rebase #28 onto control once the JSON files and the `spawn` frame are on the branch control runs.

Do not add the three Compose files together. Keep #29’s `compose.yml`. Copy environment keys and the `gz` wrapper across. Keep one default for `PX4_GZ_MODEL_POSE`.

The green `flight` check on #29 means the SITL process became healthy and `param show` matched `sim.params`. It does not mean a vehicle hovered, and it does not exercise #20, #21, or #28.

---

## Part B — Control, vision cleanup, grader, and pipeline branches

## Control and vision PR review

Review date: 2026-11-12. Repository: `ParsaKhaledi/px4_sim_ros2`. Tips are the current `origin/*` branches. GitHub recorded no inline review comments on PRs 22–27.

`origin/main` is `5eb5fde` (2026-06-29). Every branch below contains that commit and adds commits from 2026-10-08 and 2026-10-09. None of this work is on `main`. The GitHub base name `dev/px4-upgrade` is not on `origin` anymore. For PRs 22 and 26 the first commit’s parent is `5eb5fde`, and `git diff origin/main...branch` matches the GitHub file stats, so the review below is the branch as it stands.

How the tips relate:

```
main (5eb5fde)
├── feature/px4-control          PR #22 closed draft     21 commits
├── feature/flight-analysis      PR #26 closed draft      4 commits
├── feature/px4-imu-bridge-indoor-flight                  9 commits
├── feature/oakd-s2-vision       PR #20 (base of #23)
│   └── feature/vision-profiles  PR #23 closed             4 commits
│       └── refactor/vision-golden-tests  PR #24 closed    1 commit
│           └── refactor/vision-hygiene  PR #25 open draft 3 commits
│               └── refactor/vision-modules  PR #27 open draft  5 commits
├── feature/line-base            devops line + 3 commits
├── feature/line-sim             line-base + sim/vision CI fold-in
└── feature/line-control         contains px4-control, then geofence TF and TwistStamped
```

`line-control` does not contain `vision-modules` or `flight-analysis`. `line-sim` does not contain `vision-profiles`. The IMU-bridge branch is not an ancestor of `px4-control`.

---

### PR #22 — ROS 2 offboard stack for PX4 v1.17

#### 1. Summary

This branch deletes `includes/gz/microxrce_offboard.py` (537 lines) and replaces it with a ROS 2 package, `px4_control`, plus `px4_control_interfaces`. One node streams offboard setpoints. Missions call actions through a `Drone` client. Vision mode is meant to fuse external-vision horizontal position and yaw (`EKF2_EV_CTRL` 9), take height from the barometer (`EKF2_HGT_REF` 0), and leave the GPS sensor publishing with GNSS fusion off (`EKF2_GPS_CTRL` 0).

The control math is split into testable modules (`frames.py`, `trajectory.py`, `motion.py`, `vision_bridge.py`). The ROS node is still one 1091-line class. The branch is a closed draft. The plan file on the branch says the vision out-and-back was not flown, and the GPS numbers in the PR body were taken at `a123418`, before the magnetometer, GPS-off, baro-height, lidar, and geofence-parameter commits.

#### 2. Stack position

- GitHub base: `dev/px4-upgrade` (absent on `origin`). Head ref: `feat/px4-control`.
- Actual parent of `191a5b3`: `5eb5fde` (`origin/main`).
- Head: `8668a11` `docs(px4_control): record plan and where work stopped`.
- Last code commit: `7e7887f` `fix(px4_control): compare lidar and EKF2 as heights above the floor`.
- State: closed 2026-10-09, not merged. 21 commits, all by Parsa Khaledi.

#### 3. Stats

58 files, +6103 / −552. Largest new files: `px4_control/node.py` (1091), `motion.py` (621), `test/test_motion.py` (403), `frames.py` (263), `docs/px4_control.md` (256), `trajectory.py` (248), `vision_bridge.py` (213), `arming.py` (207), `geofence.py` (206).

#### 4. File-by-file commentary

**Parameter install.** `includes/gz/gz_modifications.bash` stops appending `param set-default` to `px4-rc.params`. It sources `includes/gz/params/install_px4_control_params.bash` and writes `PX4-Autopilot/px4_control_params.env`. `gz_start_px4_gz_sim.sh` sources that file with `set -a` before `make px4_sitl` (the added block sits just above the existing `PX4_GZ_MODEL_POSE` export). Stock v1.17 `rcS` applies `PX4_PARAM_*` before the airframe and before `ekf2 start`. That part of the design matches the failure mode the PR describes: v1.17 never sources `px4-rc.params`.

`install_px4_control_params` (lines 34–86) writes a shared block for both modes:

```bash
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_PARAM_NAV_RCL_ACT=1
export PX4_PARAM_UXRCE_DDS_SYNCT=0
```

Vision then adds `EKF2_EV_CTRL` (default 9, range 0..15), `EKF2_MAG_TYPE=5`, `EKF2_HGT_REF=0`, `EKF2_EV_DELAY` (default 0, range 0..300), `EKF2_GPS_CTRL=0`, `EKF2_RNG_CTRL=1`, `COM_ARM_WO_GPS=1`, `GF_MAX_VER_DIST=3.0`, `GF_ACTION=5`. GPS writes `EKF2_EV_CTRL=0`, `EKF2_HGT_REF=1`, `EKF2_GPS_CTRL=7`, and the two GPS noise params. GPS does not export `EKF2_MAG_TYPE` or `SYS_HAS_MAG`.

`px4_params.py` `expected_sim_params` (lines 32–82) mirrors that table. `EKF2_EV_CTRL` 9 is bits 0 and 3: horizontal position and yaw. Bit 1 (vertical position) stays clear, which is why `EKF2_HGT_REF` is 0 (baro). `EKF2_GPS_CTRL` 0 is there because, with GNSS position fusion on, PX4 v1.17 runs external-vision horizontal position through the bias estimator and floors its variance at `GPS_P_NOISE` squared. `GF_ACTION` 5 is Land in v1.17; 3 is Return. `readback_mismatch` (lines 89–109) skips a name that is absent from `expected`, so the pure function is safe when the GPS dict is smaller.

The node’s log line is not. `node.py` `_read_back_params` (lines 839–863) iterates every `READBACK_PARAMS` name and formats `expected[name]` whenever MAVLink returned it. `READBACK_PARAMS` ( `px4_params.py` lines 16–29) includes `EKF2_EV_DELAY`, `EKF2_RNG_CTRL`, `GF_MAX_VER_DIST`, and `GF_ACTION`. The GPS expected dict does not. A normal GPS reply therefore raises `KeyError` on the daemon thread. The `except` only catches `OSError`. `_param_report` stays `None`, and `_prepare_flight` (lines 865–871) waits until `min(timeout, 20)` then refuses to arm with “timed out reading PX4 parameters”. GPS mode on this head cannot arm. The unit test in `test_topics_and_params.py` checks the pure mismatch helper and never runs this log loop.

The read-back is one shot, started from `__init__` (line 196) on UDP 14580 (`mavlink_params.py` `read_params`, lines 138–189). A later `param set` does not clear a stored mismatch. `mavlink_params.py` decodes `MAV_PARAM_TYPE_INT32` as the int32 bit pattern (lines 84–99) because PX4 sets `PARAM_ENCODE_BYTEWISE`. Heartbeat and requests share one socket, because PX4 locks its UDP partner to the first sender. That part is covered by `test_param_request_round_trip`.

`NAV_DLL_ACT` is the other arming gate. The module docstring (`px4_params.py` lines 6–9) says the env export is enough. The PR body records the opposite experiment: the x500 airframe runs `param set-default NAV_DLL_ACT 2` after the env loop, the firmware default is already 0, and `set-default` puts 2 back. Read-back expects 0 (`px4_params.py` line 46). On a stock headless boot the node latches a mismatch and refuses to arm. The scored GPS hover used one manual `param set` and predates this read-back list. The node never sends `PARAM_SET` for `NAV_DLL_ACT` (`mavlink_params.py` lines 68–75).

**Topics and launch.** `config/topics.yaml` pins px4_msgs v1.17.0 names: `vehicle_status_v1` and `vehicle_local_position_v1`, bare names for odometry, attitude, and command ack. `topics.py` `subscription_names` (lines 88–96) also subscribes to the unversioned name when the configured leaf ends in `_vN`. `topics_config_path` (lines 48–58) prefers the ament share directory and falls back to the source tree. `gz_start_px4_control.sh` requires `ros2 pkg prefix px4_control`, defaults `ESTIMATION_MODE` to vision, and points walls at `worlds/walls/${World}.txt` when that file exists. `start_mission.sh` no longer runs `microxrce_offboard.py`. It execs `gz_start_px4_control.sh`. Anything that still expected the old script’s takeoff CLI now only starts the node.

**Frames.** `frames.py` maps ENU to NED with `R_NED_FROM_ENU` (lines 18–25) and FLU to FRD with `diag(1, -1, -1)` (line 30). Covariance uses `R @ Σ @ R.T` (`rotate_covariance`, lines 58–65). `enu_yaw_to_ned` (lines 176–182) is `π/2 - yaw_enu`: ENU yaw 0 (east) becomes NED yaw `+π/2`. `body_horizontal_to_ned` (lines 193–217) treats ROS `vy` as left, negates it to FRD right, and negates the ROS yaw rate so a positive command is counter-clockwise from above. Roll and pitch are left out on purpose; Nav2 `cmd_vel` is planar. `attitude_from_enu_quat` rejects a quaternion whose norm is under `1e-6` (line 256). `_quat_to_rot` still returns identity below `1e-12` (line 94), which that caller never reaches.

**Vision bridge.** `VisionOdometryBridge.push` (`vision_bridge.py` lines 146–199) drops a sample and marks tracking lost on `tracking_lost`, a zero quaternion plus zero position (`_is_null_pose`, lines 90–102), or a 6×6 diagonal at or above 9999 (`lost_covariance`, lines 81–87). A finite but huge variance under `max_variance` (default 25) returns `None` without marking lost (lines 166–171), so the last good sample can still be served until `timeout_s`. `current` (lines 201–213) compares the caller clock with the last accepted header stamp and does not bump `reset_counter`. `note_odom_info` increments `reset_counter` only on `new_map`, modulo 256 (lines 135–144). `reset_jump_m` is stored (lines 117–119) and never read. The node still declares `vision_reset_jump_m: 0.75`.

`node.py` `_publish_visual` (lines 432–452) writes `timestamp` and `timestamp_sample` from the RTAB-Map header in microseconds and sets `quality` to 100 on every accepted sample. With `UXRCE_DDS_SYNCT` 0, PX4 is supposed to stay on `/clock`, so a capture stamp is the right sample time and `EKF2_EV_DELAY` 0 avoids subtracting that delay twice. If those clocks diverge, every vision update is late or early by a constant. Orientation covariance is rotated by `attitude.rotation.T @ R_NED_FROM_ENU` (lines 177–182). That matches the comment (body-FRD diagonal). It assumes the ROS pose covariance’s orientation block is expressed in the parent ENU frame.

`_on_odom_info` (lines 418–430) treats a non-lost message whose covariance is 9999 as a map reset, and a lost message with that covariance as a null pose. A pose jump alone does not reset.

**Motion.** `MotionExecutive` never snaps the position setpoint onto the goal. `trajectory.step_scalar` (lines 43–78) uses a stopping speed `sqrt(2 a d + (a dt / 2)^2) - a dt / 2` so the discrete step can arrive at zero speed. `step_vector2` (lines 81–129) brakes the lateral component and clamps the combined acceleration to `a_max`. `step_yaw` takes the short wrapped path. `step_yaw_remaining` (lines 132–149) does not wrap, so `request_turn` can keep +180° and +270° on the signed ROS direction.

`request_turn` (`motion.py` lines 319–338) stores `remaining = -delta_rad` after `_begin`, which zeroes `_yaw_remaining` (line 419). The assignment order is the one that works. A moving vehicle brakes first, and the remaining angle survives `_begin_brake`. `request_hold` (lines 340–353) does the same repair for `_holding`: `_begin` clears it, then the caller sets it, then brake runs. A hold timer (`_tick_hold_timer`, lines 487–508) counts `settle_time` (default 1 s) inside 0.02 m and 3° before `duration_s` starts. `hold(10)` is therefore about 11 s of in-tolerance time, then the duration.

`request_takeoff` (lines 224–238) targets `ground_d - height` (NED down). It writes `_locked_xy` and nothing reads that field, so a takeoff does not have a separate XY lock beyond the target captured at the request. `note_vision_lost` (lines 147–168) fails the active goal and brakes to a hold, except during takeoff and land, where it only freezes `_yaw_goal`. A vision dropout during climb keeps the vertical path. The plan on this branch lists “brake, hold, then land on vision loss, including takeoff” as not done, and it records an unfiled crash in which tracking was lost on the forward leg and the vehicle climbed to about 6.5 m. `GF_MAX_VER_DIST` 3 m is the intended backstop, and it depends on the parameter actually sticking.

`_tick_land` (lines 537–551) drives the setpoint to half a metre below the recorded ground so hover thrust does not hold the vehicle in the air. After the path step it forces `velocity[2]` (down) to at least 0.40 m/s, because `LNDMC_Z_VEL_MAX` defaults to 0.25 m/s and ground contact wants at least 1.1 times that. Near the floor the position setpoint and the velocity feedforward disagree: the position can sit on the target while the velocity command stays 0.40 m/s down until `landed` is true.

`abort_and_land` (lines 247–260) is what the e2e height guard calls. `abort_brake_hold_land` (lines 262–289) is what the lidar guard calls: it opens a land goal, then immediately brakes with `_after_brake = LAND`. `_on_timer` runs `update` once, then the guard, then `update` again at the same timestamp (`node.py` lines 503–507). The second call has `dt = 0`, so the new phase is emitted and does not step.

`note_cmd_vel` (lines 355–375) ignores an all-zero twist unless a command is already active, so a zero command does not preempt a GoTo. The timeout uses the twist time, not the control tick (`command_age` in `frames.py` lines 234–241). That fixes the old bug where the hold branch divided a seconds-plus-nanoseconds pair by `1e9`.

**Node I/O.** The setpoint timer defaults to 20 Hz (`px4_control.yaml` line 4). `OffboardStreamGate` (`mode_switch.py` lines 105–142) requires at least 2 Hz for 1 s of node time, and a backwards clock resets the window. `_on_cmd_vel` subscribes to `geometry_msgs/Twist` and stamps the command with `_now_s()` (`node.py` lines 389–392 and 199–205). Nav2 on Jazzy publishes `TwistStamped`. This head does not accept that type. Commit `309ebe7` on `feature/line-control` changes the subscription; it is not on this branch.

`_on_odometry` (lines 373–387) rotates body-FRD velocity into the parent frame when `velocity_frame` is body FRD, and takes yaw rate from `angular_velocity[2]`. For a level vehicle that body-down rate matches NED yaw rate (positive clockwise from above). The published `/px4_control/odom` pose is yaw-only (`_publish_state`, lines 628–633); roll and pitch are dropped.

`_check_e2e_height` (lines 538–555) is off unless `e2e_height_tolerance_m` > 0. The first sample whose negated NED z or setpoint z leaves `/ground_truth/odom` z by more than that tolerance aborts and lands. `HeightGuard.fault_to_abort_s` is always 0 on that path because fault time and abort time are the same sample (`e2e_height.py` lines 46–52). The log line still prints the gap.

`LidarHeightGuard` (`lidar_height.py`) stays disarmed until a reading is inside min/max, `signal_quality != 0`, and the raw range is above 0.5 m. Tilt compensation is `range * R[2, 2]`, then minus `lidar_mount_offset_m` (default 0.19: base_link 0.24 m above the model origin, sensor 5 cm below base_link). The EKF side is local down relative to the last landed sample (`node.py` `_ekf_height_above_ground`, lines 557–564), not `dist_bottom`. Before any landed sample it uses `-down`, which treats local z = 0 as the floor. A sustained gap over 0.3 m for more than 0.5 s calls `abort_brake_hold_land`. No `distance_sensor` logs once after 1 s and does nothing (`absence`, lines 156–166).

`_exec_land` (lines 947–975) calls `goal_handle.succeed()` inside `_wait_goal` before disarm. If disarm then fails, `result.success` becomes false while the goal was already marked succeeded. The comment at lines 971–972 says so. A client that trusts the action status and a client that trusts `result.success` disagree.

`_exec_goto` feedback (lines 1017–1025) always publishes `distance_remaining = 0` and `yaw_error_rad = 0`. `_start_goto` (lines 1038–1077) converts a body offset with `body_offset_to_enu` and an absolute ENU yaw with `enu_yaw_to_ned`. Wall rejection samples the straight ENU path at 21 points (`geofence.path_rejection`, lines 181–206). With an empty wall file, which is the default (`px4_control.yaml` `wall_segments_file: ""`), `load_wall_file` returns no segments and every goal is accepted at unlimited speed (`geofence.py` lines 32–37 and 109–110). Segments are local ENU text (`x1 y1 x2 y2`). There is no world-to-spawn transform on this branch. `max_speed_along` (lines 128–158) limits speed by clearance along the motion (`clearance / closing`), so a diagonal does not use the perpendicular gap as if it were the stopping distance.

`Drone._wait_future` (`drone.py` lines 144–161) starts the sim-time budget at the first positive `/clock` sample. A deadline computed at time 0 would expire when `/clock` jumps. `examples/out_and_back.py` is `takeoff(2)`, `hold(10)`, `move_forward(0.3)`, `turn(180)`, `move_forward(0.3)`, `land()`.

`grading.grade_hover` flags a vision hover whose every height error is within 1 mm as `suspect`. GPS mode returns `ok`. Nothing in the node calls it; it is a helper for a scorer that is not in this PR.

#### 5. Review verdict

Strengths:

- The failure history is in the code, not only in the PR text: bytewise params, one MAVLink socket, sim-time deadlines, covariance rotation, RTAB-Map’s 9999/null-pose convention, land-detector vertical speed, and the short-path versus signed-turn split.
- Pure modules have unit tests (`test_motion.py`, `test_frames.py`, `test_vision_and_modes.py`, `test_lidar_height.py`, `test_geofence.py`, `test_e2e_height.py`). The PR body says 41 pytest tests passed without Gazebo. This review did not re-run them.
- GPS fusion is off in vision mode for a reason that matches the PX4 v1.17 bias-estimator behaviour, and the navsat sensor is left in place.

Risks, in the order that blocks a flight:

- Headless arming still depends on `NAV_DLL_ACT` 0, and the airframe `set-default` runs after `PX4_PARAM_*`. Read-back expects 0 and never retries. The node will refuse to arm on an unmodified x500 boot.
- GPS mode’s read-back logger raises `KeyError` on four parameters the GPS profile does not list. GPS mode cannot arm on this head.
- The only scored flight in the PR body is a GPS hover at `a123418`, before the commits that changed magnetometer policy, GPS fusion, height reference, the geofence ceiling, and the lidar guard. No vision flight is attached. The plan’s crash note (lost tracking, climb to about 6.5 m) has no report in the tree.
- Vision loss during takeoff or land does not brake and land. Height in vision mode is the barometer. The 3 m ceiling only helps if `GF_MAX_VER_DIST` survived startup.
- `/cmd_vel` is `Twist`. Nav2 on this stack publishes `TwistStamped`.
- Walls do nothing until a per-world text file exists, and that file is not transformed into the EKF frame. `578017e` on `line-control` starts that work and is not here.
- `node.py` holds the action servers, the param thread, the lidar and e2e guards, and the setpoint loop. `_param_report` is written off the executor thread with no lock.
- Land can report action success before disarm fails.

Merge readiness: not ready. Closed draft, vision flight absent, and the two arming bugs above fail the modes the package claims to support. Rebase onto whatever replaces `dev/px4-upgrade` is still open; the plan says PR #19 should own the headless `NAV_*` params so this script does not write them.

#### 6. Suggested follow-ups

1. Make `_read_back_params` log only names present in `expected`, catch `Exception`, and set `_param_report` in every path. Add a GPS test that feeds a full MAVLink reply including `GF_ACTION`.
2. Apply `NAV_DLL_ACT` after the airframe, or stop expecting 0 until that post file exists. Re-read until the values match or a deadline fires, and keep the last good read if PX4 is still booting.
3. On vision loss in every phase, including takeoff, brake, hold, and land. Add a regression that a lost `OdomInfo` during takeoff produces a land goal.
4. Subscribe to `TwistStamped` and time out on the header stamp, which `309ebe7` already does on `line-control`. Keep a zero stamp from looking infinitely old.
5. Bring the spawn-frame wall alignment from `578017e` back onto this branch, or drop wall limiting until that lands.
6. Split `node.py` only after the arming and vision-loss behaviour is stable. The plan’s cleanup list is premature while GPS mode cannot arm.
7. Fly vision out-and-back on the cpu profile and attach the log. The GPS table in the PR body is not evidence for this head.

---

### PR #23 — selectable cpu, full, and hw profiles

#### 1. Summary

This branch adds a third vision profile, `hw`, moves `full` stereo from the 640×400 calibration bin to the native 1280×800 OV9282 size, and changes the default `VISION_PROFILE` from `full` to `cpu`. Profile selection, allowed resolutions, and the two warnings live in `includes/gz/oakd_s2/geometry.py`. RTAB-Map keys move into `includes/gz/startFiles/rtabmap_profiles/{cpu,full,hw}.ini`, loaded with the launch argument `cfg:=`. Camera-mode keys stay on `args` / `odom_args` because one ini cannot describe both stereo and RGB-D.

The health floors for `full` at 1280×800 and for all of `hw` are still the old or interpolated numbers. The docs say that in the same sentences that publish the numbers.

#### 2. Stack position

- GitHub base: `feat/oakd-s2-vision` (`8a27573`). Head: `feat/vision-profiles` (`d5734d3`).
- Four commits on top of the OAK-D S2 branch: select the three profiles (`463c639`), default to cpu (`3dbb7e6`), find `rtabmap_params.py` from the container mount (`362da81`), print the no-GPU warning only from the Gazebo render (`d5734d3`).
- State: closed 2026-10-09, not merged. The parent chain sits on `main` through PR #20’s six commits.

#### 3. Stats

22 files, +1060 / −238 versus `origin/feature/oakd-s2-vision`.

#### 4. File-by-file commentary

**Profile table.** `geometry.py` lines 86–117 define the sizes. `FULL_PROFILE` (lines 186–194) is 1280×800 stereo, 640×480 color, 30 Hz cameras, 200 Hz IMU. `CPU_PROFILE` (lines 195–203) is 320×200, 320×240, 10 Hz, 100 Hz. `HW_PROFILE` (lines 204–212) is the native stereo size at 15 Hz with the 200 Hz IMU. `ALLOWED_STEREO_RESOLUTIONS` (lines 110–114) is only those three 16:10 scales of the 640×400 calibration: 1280×800, 640×400, 320×200. `1280×720` is rejected so a crop cannot move the principal point.

`profile_from_env` (lines 289–313) treats a missing or blank `VISION_PROFILE` as `cpu`. Unknown names raise. `CAM_RATE_HZ` and `IMU_RATE_HZ` override rates. `_stereo_size` (lines 263–286) prefers `CAM_STEREO_RES` (`1280x800` form) and rejects a width/height pair that disagrees with it. `_env_int` truncates `320.7` to 320 (lines 220–227). That is intentional and easy to miss.

`scale_intrinsics` keeps the field of view, so `P[3] / P[0]` stays `-0.075` at every allowed size (`stereo_tx` uses the scaled fx and the 7.5 cm baseline). Both cameras still share the left calibration. The right yaml is not rendered. The checked-in `OakD-Lite` SDFs in this diff change width, height, and the doubled intrinsics (fx about 813.25, cx about 611.55). Those checked-in files are the full profile. A container that skips `render_oakd.py` at start serves 1280×800 while a default `cpu` RTAB-Map session expects 320×200.

**Warnings.** `no_gpu_warning` (lines 341–357) returns a line for `full` and `hw` when neither `/dev/dri/renderD*` nor a successful `nvidia-smi` is present. `cpu` stays quiet. It does not reject the profile. `render_oakd.gazebo_startup_warnings` (lines 495–514) adds that line only when `write_sdf` is true. `--urdf-only` and the RTAB-Map process do not print it. That is the `d5734d3` fix: the Rtabmap container often has no `/dev` and would otherwise warn on every start.

The wording is still aimed at a simulation host. `hw` is the real-camera profile, and the warning tells you to set `VISION_PROFILE=cpu` whenever the process that writes SDFs sees no GPU. A LattePanda or Orin NX running the same renderer, with the OAK-D doing depth on-device, hits that path and gets the cpu suggestion.

`sub_hd_warning` (lines 360–375) fires for `full` or `hw` below 1280×720. The only allowed size that passes is 1280×800, so a `full` run at the old 640×400 calibration warns. `cpu` at 1280×800 does not warn. `stereo_info_relay.py` prints the sub-HD line and does not call `no_gpu_warning`.

**RTAB-Map inis.** `rtabmap_params.py` `INI_DIR` (line 25) is `includes/gz/startFiles/rtabmap_profiles` next to the package. `parse_rtabmap_ini` (lines 39–59) reads `[Core]`, turns `Vis\MaxFeatures` into `Vis/MaxFeatures`, and ignores other sections. `camera_overrides` (lines 67–80) keeps `Stereo/MaxDisparity` 64 on cpu stereo only. `full` and `hw` leave the master default of 128. The tuning doc in this PR says that at 1280×800 the same default 128 is a farther minimum depth, because fx doubles. `hw` also sets `Vis/MaxFeatures` 600, `OdomF2M/MaxSize` 1500, and `Odom/VisKeyFrameThr` 40. The doc says those three are not measured.

`rtabmap_profile.sh` is what the start scripts source. The unit test builds a fake `/volume/includes/gz` tree and checks the cfg path ends in `startFiles/rtabmap_profiles/cpu.ini` (`test_oakd_s2.py` around the volume fixture). `OAKD_S2_DIR` can point elsewhere. An unset profile resolves to cpu.

**Health log.** `HealthCheck/rtabmap_health_log.py` `profile_feature_defaults` maps cpu to 40 features and 15 inliers (measured at 320×200), full to 120 and 20 (measured when full was 640×400), and hw to 80 and 20 (not measured; 80 is “about the same share” of 600 features as 120 is of 1000). An explicit `VISION_MIN_*` still wins. The default profile change means a health log that used to apply the full floors now applies the cpu floors unless `VISION_PROFILE` is set.

**Compose and env.** `docker-compose-px4.yml` default `VISION_PROFILE` becomes `cpu` on the PX4, state-publisher, and RTAB-Map services, and it passes `CAM_STEREO_RES`. `.env` and `.env.example` both change. Anyone who relied on an unset profile to mean the old 640×400 “full” spec now gets the cpu sim profile, and opting back into `full` is a different resolution than the previous full.

#### 5. Review verdict

Strengths:

- One allow-list for stereo size, with a stated reason for rejecting a 16:9 crop.
- The GPU warning is confined to the process that renders Gazebo cameras.
- The unmeasured floors are labeled in `.env`, the README, and `docs/rtabmap_tuning.md` instead of being presented as flight results.
- Tests cover the three names, case folding, disagreeing `CAM_STEREO_RES`, the sub-HD warning, and the GPU warning’s caller.

Risks:

- Default `cpu` plus checked-in full SDFs is a split brain if render-on-start is skipped.
- `full` at 1280×800 changes disparity search depth and has no new RTF or health measurement. The old 0.125 RTF figure is for 640×400.
- `hw` on a machine with no DRM node and no `nvidia-smi` is told to switch to `cpu`.
- `hw` feature and inlier gates can fail a healthy onboard run, or pass a weak one, because 80 and 20 were not measured.

Merge readiness: acceptable as a draft API for the cleanup PRs stacked on it. Do not treat `full` or `hw` as qualified flight profiles until a GPU run and an onboard run replace the inherited floors. Closed, not merged, and still waiting on the OAK-D base (PR #20) and the DevOps base the vision plan names.

#### 6. Suggested follow-ups

1. Render the SDF for the active profile in every start path, and fail the start if the written width does not match `profile_from_env`.
2. Split the no-GPU sentence so `hw` is not told to become `cpu` when the host is the onboard computer and Gazebo is not the consumer.
3. Re-measure `Vis/MaxFeatures`, inliers, and minimum depth at 1280×800 before CI gates on those floors.
4. Keep `.env` and `.env.example` on the same `VISION_PROFILE` default, which this PR does, and call out the default change in the merge note so a GPU user does not silently drop to 320×200.

---

### PR #24 — golden snapshot tests

#### 1. Summary

One commit, `de0a33a`, adds `includes/gz/oakd_s2/test_golden.py` and 51 text fixtures under `includes/gz/oakd_s2/fixtures/golden/`. The test renders SDF, URDF, RTAB-Map shell and stderr, health thresholds, and the stereo relay `K`/`P` in memory. `UPDATE_GOLDEN=1` rewrites the fixtures. A mismatch fails with a unified diff. The test refuses to write a path that escapes the fixture directory, and it hashes the checked-in SDF and URDF before and after so a golden run cannot overwrite the models Gazebo ships.

#### 2. Stack position

- Base: `feat/vision-profiles` (`d5734d3`). Head: `refactor/vision-golden-tests` (`de0a33a`).
- Parent of the vision stack. PR #25’s base is this head.
- State: closed 2026-10-09, not merged. The vision plan says 102 tests passed and review had not started.

#### 3. Stats

53 files, +5098 / −1. The code file is 307 lines. The rest is generated SDF (about 207–214 lines each), URDF (76 lines), and short JSON or ini logs. `.gitignore` is the one-line deletion.

#### 4. File-by-file commentary

`test_golden.py` builds a producer map (lines 190–206):

- `sdf/{cpu,full,hw}/{default,translated,pitched}/{stereo,rgbd}.sdf` — 18 files. Translated sets `CAM_X/Y/Z` to 0.20, −0.01, 0.30. Pitched sets `CAM_PITCH_DEG` to 25.5.
- `urdf/{profile}/{mount}.urdf` — 9 files. `render_urdf` does not take a profile. The three blobs per mount are identical (three distinct git hashes, each stored three times). A profile-only change still updates nine URDF fixtures, and a real URDF change has to be reviewed three times.
- `rtabmap/{profile}/{stereo,rgbd,rgbd-wrapper}.{shell,log}` — 18 files. The subprocess clears `VISION_PROFILE` and the `CAM_*` overrides, then the checkout path to the ini directory is rewritten to the token `rtabmap_profiles` (`_portable`, lines 117–119). `test_rtabmap_fixtures_use_a_portable_ini_path` asserts the repo path is absent.
- `health/{profile}.json` from `thresholds_from_env` with the `VISION_*` overrides removed.
- `relay/{profile}.json` from `rectified_k` and `rectified_p(right=True)`, the two calls the relay uses. No ROS node starts.

`setUpModule` (lines 239–248) writes fixtures only when `UPDATE_GOLDEN=1`, and only after `Path.is_relative_to` says the target stays under `fixtures/golden`. `test_fixture_files_match_the_snapshot_set` fails if a file is added or removed without a producer. `test_zzz_checked_in_models_were_not_rewritten` compares SHA-256 of `STEREO_SDF`, `RGBD_SDF`, and `URDF_PATH` against the hashes taken at import.

The SDF goldens freeze the PR #23 resolution change. Updating intrinsics or clip planes requires regenerating 18 SDFs and reading the diff. That is the point of the test, and it is also why PR #25’s constant rename has to keep the rendered text stable.

#### 5. Review verdict

Strengths:

- The generator and the checker are the same functions, so a fixture cannot drift from a hand-edited expected file without `UPDATE_GOLDEN`.
- Portable ini paths and the checked-in-model hash make the test safe to run in a developer tree.
- Health and relay outputs are small JSON, which is the right grain for those two contracts.

Risks:

- Triplicated URDF fixtures hide the fact that the URDF ignores `VISION_PROFILE`.
- A golden failure prints the full unified diff of a 200-line SDF. Reviewing an intentional change means regenerating and then reading 18 near-copies.
- The test imports `geometry` and `render_oakd` by putting `oakd_s2` on `sys.path`. PR #27’s split has to keep that import surface, which it does by re-exporting.

Merge readiness: ready as a test-only commit once you accept the fixture volume. It does not change runtime behaviour. It is closed and unmerged only because the stack under it is unmerged. Run it once on the stacked head before merging #25 or #27; those PRs claim the snapshots still match.

#### 6. Suggested follow-ups

1. Snapshot each URDF mount once, not once per profile, unless `render_urdf` grows a profile argument.
2. Keep `UPDATE_GOLDEN=1` out of CI. The path check is in the test, and a CI env leak would rewrite the tree.
3. When #27 splits modules, run this file unchanged before editing fixtures.

---

### PR #25 — hygiene, named constants, and docstrings (cleanup A)

#### 1. Summary

Three commits on top of the golden tests: drop stale comments, unused imports, and two dead start scripts (`ffc10c7`); name clip, bias, and housing colors (`035b9e2`); add docstrings (`d78960e`). Runtime behaviour is almost unchanged. The rendered camera text is supposed to stay on the golden snapshots, because the new names replace literals that already had those values.

This is an open draft. The vision plan says shellcheck findings on the touched scripts went from 12 to 3, and that the per-folder READMEs are a later PR.

#### 2. Stack position

- Base: `refactor/vision-golden-tests` (`de0a33a`). Head: `refactor/vision-hygiene` (`d78960e`).
- Open draft. PR #27’s base is this head.

#### 3. Stats

14 files, +258 / −71.

#### 4. File-by-file commentary

**Dead scripts.** The diff deletes `includes/gz/startFiles/start_realsense.sh` (a `realsense2_camera` launch) and `start_state_publisher.sh` (an owl TF launch under `volume/Params`). Compose and `tmux-session.sh` call `gz_start_state_publisher.sh`, which this PR keeps. A search for the deleted filenames on this tip matches only the `gz_` script, because that name contains the old suffix as a substring. Nothing invokes the deleted files.

**State publisher.** `gz_start_state_publisher.sh` drops an unused `WORKDIR`, quotes the `cd`, and fails the `cd` instead of launching from the wrong directory. The trailing “Update addresses” comment goes away. `robot_state_publisher.launch.py` drops a duplicated import block (`DeclareLaunchArgument`, `get_package_share_directory`, `os`) that the file no longer uses. The node still builds the robot description with `Command`.

**Constants.** `geometry.py` names `HOUSING_AMBIENT_RGBA` / `HOUSING_DIFFUSE_RGBA`, the three clip ranges (`STEREO_CLIP_*` 0.2–30 m, `COLOR_CLIP_*` 0.08–50 m, `DEPTH_CLIP_*` 0.2–12 m), and the bias stand-ins `GYRO_BIAS_STDDEV_RAD_S = 1e-4`, `ACCEL_BIAS_STDDEV_M_S2 = 1e-3`. The comments say the bias figures are not from a BNO086 datasheet; they exist so the sim is not bias-free. SDF text that inlines these numbers must stay byte-identical to the goldens.

`_env_float` moves up so the profile helpers can share it. `_resolution_text`, `_hd_floor_text`, and `_real_use_stereo_text` replace the literals `1280x720` and `1280x800` in the error strings. `_allowed_stereo_text` now says the HD floor string “is rejected because it would crop” the real-use size. Today those helpers return `1280x720` and `1280x800`, so the sentence matches the old one. The HD floor is a warning threshold, and 1280×720 is also the example of a rejected crop. One helper now serves both sentences. If you later set the warning floor to a size that is not the rejected crop, the error text will name the wrong resolution.

**Health-check dead branch.** In `HealthCheck/check_vision_pipeline.py` the TF loop already computes:

```python
expected = geo.optical_origin_in_base(sensor, mount) if sensor != "imu" else geo.optical_origin_in_base("imu", mount)
```

The following `if sensor == "imu": expected = optical_origin_in_base("imu", mount)` assigned the same value and is removed. The ternary that remains is still two calls to the same function. Behaviour is unchanged, and the cleanup is one assignment short of a single call.

**Docstrings.** The bulk of the +258 is one-line docstrings on `rtabmap_health_log.py`, `vision_rate_probe.py`, `check_vision_pipeline.py`, `geometry.py`, `render_oakd.py`, and the tests. They describe the function that is already there. `TrackingLog` and the rate-probe callbacks pick up the same treatment. No threshold default changes in this PR.

**Start-script comments.** `gz_start_rtabmap_stereo.sh` documents `RTABMAPVIZ` and replaces the literal `0.075` in a comment with `BASELINE_M`. `ResetCountdown` is described as 1 in the ini.

#### 5. Review verdict

Strengths:

- The deletions are unused entry points, and the live `gz_start_state_publisher.sh` path is kept.
- Named clip and bias constants give the golden SDFs a single source for numbers that were previously literals in the renderer.
- Docstrings are short and sit on functions a later split will move.

Risks:

- `_hd_floor_text()` inside the “this size is rejected” sentence couples a warning threshold to an allow-list error. It is correct only while the floor stays 1280×720.
- The IMU expected-origin ternary is still redundant. A later edit that special-cases the IMU on one side only will not do what the comment says.
- This PR does not regenerate or edit goldens. Any constant that was not already the rendered number fails `test_golden.py`. That test is the merge check.

Merge readiness: ready to merge after `test_golden.py` and `test_oakd_s2.py` pass on this tip, and after you accept the stacked base. It is still an open draft, and it should not land ahead of #23 and #24.

#### 6. Suggested follow-ups

1. Replace the IMU ternary with one `optical_origin_in_base(sensor, mount)` call.
2. Keep the rejected-crop example as its own constant, separate from `HD_MIN_WIDTH` / `HD_MIN_HEIGHT`.
3. Run the golden test before review sign-off. A docstring-only claim is cheap to verify and expensive to get wrong in the SDF.

---

### PR #26 — offline flight-log grader

#### 1. Summary

`flight_analysis` reads a PX4 `.ulg` (through pyulog) and optional TUM ground truth, segments the command track into legs, and writes a JSON report of overshoot, settling, stopping distance, tracking error, estimation error, tilt, actuator saturation, body-rate spectrum, height divergence, estimator flags, vision delay, and a ground-truth leak check. Limits come from `e2e_limits.py` at the repo root: process environment, then `.env`. `.env.example` is never a fallback.

The tool has not been run on a real v1.17 SITL log. The plan on the branch says so, and it says the `spawn.json` producer was cancelled before it was pushed.

#### 2. Stack position

- GitHub base: `dev/px4-upgrade`. Actual parent of `bdf1efa`: `5eb5fde` (`origin/main`).
- Head: `4536020` `Add the flight mechanics and control plan.` Last code commit: `f1cd670` `Accept a null PX4 clock offset and copy its spread into the report.`
- Four commits. Closed 2026-10-09, not merged. Not contained in `line-control`.

#### 3. Stats

26 files, +4949 / −0. Largest: `metrics.py` (683), `grade.py` (439), `test_metrics.py` (435), `log.py` (392), `test_spawn.py` (349), `analyze.py` (331), `spawn.py` (324), `segment.py` (255), `tum.py` (240).

#### 4. File-by-file commentary

**Limits.** `e2e_limits.py` `LIMIT_FIELDS` (lines 22–44) is 21 `E2E_*` names with no numeric defaults in the dataclass. `load_limits` (lines 112–143) fills from `.env` first and lets the process environment overwrite. A missing name raises `E2ELimitError` listing every missing key. `_parse_value` (lines 165–180) rejects non-finite and negative values, and requires `E2E_MAX_RETRIES` to be a whole number. `repo_root` is `Path(__file__).parent` (lines 80–83), so the module must stay at the repository root and the process must have that directory on `PYTHONPATH`. The file’s own docstring points at `tests/e2e/grading.py` “once that file is on the branch”. It is not on this branch. `feature/line-base` has a `tests/e2e/grading.py`; this PR does not import it.

**Spawn and clock.** `spawn.resolve_spawn` reads `spawn.json` next to the log unless you pass `--spawn-xyz` and `--spawn-yaw`. CLI wins over the file (`_prefer`). `_offset_value` (lines 214–225) treats a missing `px4_offset_s` as “no offset” and a JSON `null` as `explicit_null`. `f1cd670` is that distinction. `clock_quality_warnings` (lines 278–293) warns when the spread (median gap minus minimum gap) is over 0.02 s or the sample count is under 50, and the offset is still used.

`analyze._resolve_truth` (lines 137–194): a TUM file is moved from Gazebo world ENU into the spawn frame, then ENU/FLU to NED/FRD. If `px4_offset_s` is missing or null, `align_tum_to_ulog` estimates the shift from up-velocity during the climb. Otherwise `alignment_from_px4_offset` (`tum.py` lines 125–149) adds `-px4_offset_s` to each TUM stamp, because the file’s convention is ROS sim time minus PX4 boot time. The report stores both `applied_offset_s` and `px4_offset_s = -applied_offset_s` (`analyze.py` lines 197–216). A logged `vehicle_local_position_groundtruth` is ignored when a TUM file is present, and the report says so.

`frames.world_enu_to_spawn_enu` (lines 97–132) uses yaw only. The local horizontal axes are `R_z(-spawn_yaw)` of the world delta. Position conversion in `enu_flu_to_ned_frd` (lines 62–94) is `(north, east, down) = (y, x, -z)`, and the attitude is `q_enu_to_ned ⊗ q_enu ⊗ q_flu_to_frd`. The comment states `yaw_ned = π/2 - yaw_enu`, the same relation `px4_control` uses. The two copies are not a shared module.

**Log decode.** `ULogLoader` (`log.py` lines 56–83) skips `multi_id` other than 0 and keeps the first chunk of a repeated topic name. `flight_log_from_arrays` sorts by `timestamp` microseconds. `command_track` prefers `trajectory_setpoint` and falls back to `vehicle_local_position_setpoint`. Missing both is a hard error. Tests call `flight_log_from_arrays` so pyulog is optional until you point the CLI at a file. Field names for a v1.17 log are unverified; the plan lists that as open.

**Metrics.** `segment_command` splits the setpoint into takeoff, land, translate, yaw, and hover. `measure_flight` scores each leg’s overshoot, settling time, and stopping distance against the estimate, and it keeps tracking error (estimate versus command) separate from estimation error (estimate versus truth). `_return_distance` (lines 551–570) is the horizontal distance from the end of the leg before the first translate to the end of the last translate, on the estimate. Two translates are required; a one-leg hop does not get a return check.

`_leak` (lines 504–548) looks at hover windows where `cs_ev_pos` is active at least half the time. If the peak position error versus ground truth is at or under `VISION_LEAK_M` (1 mm), the hover is `suspect`. No ground truth, no hover, or no estimator flags means `checked: false`. A single window under 1 mm returns immediately, so a later worse window does not clear the flag. That matches the intent in `px4_control/grading.py`, with the extra requirement that vision position fusion was actually on.

**Grade.** `grade` (lines 15–26) sets `passed` from every check. `_check` (lines 408–431) forces `passed: True` when `skipped` is true, so a missing topic does not fail the run. `_height_check` fails on the first estimate-versus-truth height gap over `E2E_HEIGHT_TOLERANCE_M`. `_tilt_check` uses the logged `MPC_TILTMAX_AIR` when it is present. `_rates_check` (lines 326–349) and `_vision_check` (lines 352–376) and `_flags_check` (lines 379–399) always pass. They report RMS, dominant frequency, delay percentiles, and whether flags were logged. A violent oscillation does not fail the grade. The reason string says there is no E2E oscillation limit.

`plots.py` writes position, yaw, tilt, and error figures when matplotlib imports. The plan says the plots have not been looked at on a real log.

#### 5. Review verdict

Strengths:

- Limits have one list, no hidden defaults, and a hard error when `.env` and the environment are both missing a key.
- Null clock offset is a documented state, not a crash, and the climb-edge fallback is labeled in the report.
- Tracking error and estimation error are separate, which is the distinction you need when the setpoint itself is wrong.
- Synthetic tests cover alignment, spawn parsing, frames, limits, and metrics. The plan says 42 unit tests. This review did not re-run them.

Risks:

- No real `.ulg` has been decoded. Topic and field names (`trajectory_setpoint`, `cs_ev_pos`, quaternion order) can be wrong for the v1.17 logger and the tests would still pass.
- `e2e_limits.py` at the repo root is an import-path accident. A second copy of the same 21 names already exists on the DevOps line.
- Body rate, vision delay, and estimator flags cannot fail a run. A reader of `passed: true` can miss them.
- The ENU/NED helpers duplicate `px4_control/frames.py`. A sign fix in one tree will not reach the other.
- TUM alignment by vertical-velocity correlation needs a climb. A log that starts in hover, or a null offset plus a flat TUM file, raises `TimeAlignmentError` or locks onto the wrong peak. That is the right failure if you see it; it is untested on the flight you care about.

Merge readiness: ready as an offline tool with synthetic coverage, not as a gate on CI flights. Closed draft. Wire it to a saved log before anyone treats `passed` as a flight result. It does not depend on PR #22 to merge, and it does not share code with it yet.

#### 6. Suggested follow-ups

1. Run one v1.17 SITL `.ulg` plus a TUM file and fix the field names the plan already flags.
2. Move `e2e_limits.py` next to the package, or import the copy the DevOps branch puts under `tests/e2e`, so the 21 names exist once.
3. Decide which informational checks are allowed to fail. If body-rate RMS stays informational, keep it out of the top-level `passed` story in the README so a green report is not read as “the vehicle was quiet”.
4. Share the ENU/NED conversion with `px4_control` after both land, or add one cross-test vector that both modules must match.

---

### PR #27 — split geometry and render modules (cleanup B)

#### 1. Summary

Five commits on the hygiene tip move `geometry.py` and `render_oakd.py` apart without a stated behaviour change. `geometry.py` becomes a re-export facade. New modules: `env.py`, `hardware.py`, `profiles.py`, `gpu.py`, `intrinsics.py`, `frames.py`, `sdf.py`, `urdf.py`, `x500_patch.py`. `camera_overrides` in `rtabmap_params.py` becomes a Python 3.12 `match`. `pyproject.toml` sets `requires-python = ">=3.12"`. The vision plan says the golden snapshots are unchanged at `5dcf589`. A docs commit, `a06ef68`, records where the vision work stopped.

This is an open draft. The plan’s head for the code split is `5dcf589`; the branch tip is the plan commit on top of it.

#### 2. Stack position

- Base: `refactor/vision-hygiene` (`d78960e`). Head: `refactor/vision-modules` (`a06ef68`).
- Open draft. Five commits: shared env reader (`3cb8ef5`), geometry split (`978c4c6`), renderer split (`45576d1`), 3.12 dataclasses and `match` (`5dcf589`), plan (`a06ef68`).

#### 3. Stats

14 files, +1615 / −1210. `geometry.py` shrinks by about 820 lines of implementation and keeps the import surface. `render_oakd.py` shrinks by about 492 lines.

#### 4. File-by-file commentary

**`env.py`.** `env_str`, `env_int`, and `env_float` are the single empty-means-unset readers. `env_int` still does `int(float(raw))`, and the docstring says `"320.7"` becomes 320 because that was `_env_int`. `env_str` returns the raw string, not a stripped one, after rejecting whitespace-only text. Callers that compared stripped profile names still strip themselves.

**Facade.** `geometry.py` on this tip imports and re-exports the public names, plus private helpers the tests already call (`_env_int`, `_env_float`, `_nvidia_smi_ok`, `_allowed_stereo_text`, `_color_full`, `_add`). `import geometry as geo` in `test_oakd_s2.py`, `test_golden.py`, `stereo_info_relay.py`, and `rtabmap_params.py` keeps working. The module docstring says the numbers still have one home; they now live in `hardware.py` and are re-exported. Two homes exist if a later edit assigns a constant in the facade.

**Split boundaries.**

- `hardware.py` holds the datasheet constants, clip planes, IMU noise, housing colour, and `HousingSpec`.
- `profiles.py` holds `VisionProfile`, the three profiles, resolution parsing, and the two warnings.
- `gpu.py` holds `nvidia-smi` and `/dev/dri` detection.
- `intrinsics.py` holds scaling, `K`, `P`, and FOV.
- `frames.py` holds the mount, the optical rotation, and the quaternion helpers. This `frames.py` is the camera frame, not the PX4 ENU/NED `frames.py` in `px4_control`.
- `sdf.py`, `urdf.py`, and `x500_patch.py` are what `render_oakd.py` used to contain. `render_oakd.main` still parses `--urdf-only` and `--patch-x500` and calls `gazebo_startup_warnings`.

**3.12.** `rtabmap_params.camera_overrides` replaces the if/elif chain with `match camera`. The pairs for stereo, rgbd, and rgbd-wrapper are the same tuples, including cpu-only `Stereo/MaxDisparity` 64. The wildcard case raises the same `ValueError`. `pyproject.toml` requires 3.12 and sets Ruff’s target. ROS 2 Jazzy on Ubuntu 24.04 is 3.12, so the container matches. A host venv on 3.11 cannot import this package. The rest of the repo is not pinned to 3.12 by this file; only `oakd_s2` is.

`rate_text` moves to `profiles.py` and the startup log calls it. Whole-number rates still print as ints, which is what the golden stderr snapshots contain. A formatting change here fails `test_golden.py` on the ini log lines.

**Plan file.** `docs/plans/vision.md` is status, not behaviour. It records #20, #23, #24, #25, and this split as done and unmerged, and it leaves a “PR C” (health-log and README cleanup) unstarted. The table’s tip for this PR is `5dcf589`. The branch tip is one docs commit later.

#### 5. Review verdict

Strengths:

- The facade preserves `import geometry as geo`, so #24’s tests do not need a rewrite to land this split.
- `env.py` documents the truncation behaviour instead of leaving it as a surprise in a private helper.
- The `match` is a straight transcription of the camera table.

Risks:

- Re-exporting private names freezes the test coupling the split was meant to loosen. Renaming `_add` becomes an API change.
- `requires-python = ">=3.12"` is a new floor for anyone running the oakd tests outside the Jazzy image.
- A move this large can drop a constant or swap a keyword argument and still look green if goldens are not run. The plan asserts they were run. The assertion is not a CI artifact in the PR.
- Two unrelated `frames.py` modules will confuse the next person who greps for yaw conversion.

Merge readiness: ready as a refactor after `test_golden.py` and `test_oakd_s2.py` pass on `a06ef68` with no fixture diff. Open draft. Land it only on top of #25, and do not mix it with a behaviour change in the same review.

#### 6. Suggested follow-ups

1. Run the golden test and attach the pass line. If a fixture changes, that change is a behaviour change and belongs in its own commit.
2. Stop exporting helpers that start with `_` once the tests import the module that owns them.
3. Leave Python 3.12 as the oakd floor only if every start script runs inside the Jazzy image. Say that in `includes/gz/oakd_s2/README.md`.
4. Rename the camera module if you touch it again (`oakd_frames.py`, or keep the facade and do not add a second `frames.py` next to `px4_control`).

---

### `origin/feature/line-base`

#### 1. Summary

This is the DevOps and CI line, not a control or vision feature branch. Relative to `origin/main` it is 30 commits ahead, and almost all of that is `feature/devops-compose-health-ci`. Three commits are unique versus that DevOps tip. They adjust the headless hover CI after the param and viewer work, and they mark this branch as the first pipeline branch.

#### 2. Stack position

- Tip: `b0d430e` `fix(ci): expect the RTAB-Map viewer to stay closed`.
- Merge-base with `main`: `5eb5fde`. Ahead 30, behind 0.
- Does not contain `px4-control`, `vision-modules`, or `flight-analysis`.
- `line-sim` and `line-control` both use pieces of this history; neither has this tip as a pure ancestor (`line-control`’s merge-base with `line-sim` is `89c2d8e`).

#### 3. Stats

Versus `origin/feature/devops-compose-health-ci`: 8 files, +204 / −40. The three commits are `89689a4`, `cad0115`, `b0d430e`.

#### 4. File-by-file commentary

The unique delta touches `tests/e2e/grading.py` (+110 in the combined stat), `tests/test_hover_flight.py` (+27), `tests/test_px4_params.py` (+5), and `tests/test_compose_config.py`. `89689a4` makes the param-show sample match `sim.params`. `b0d430e` expects the RTAB-Map viewer to stay closed, consistent with `RTABMAPVIZ=false` on the vision branches. `cad0115` is the branch marker for CI.

This is CI expectation maintenance on the DevOps stack. It does not change the offboard controller or the OAK-D geometry.

#### 5. Review verdict

Useful as the pipeline base the later line branches claim to extend. The three unique commits are small and test-facing. Merge readiness of the whole 30-commit branch is the DevOps PR’s question (PR #19 / #29), not this review. Do not look here for the px4_control or vision-profile work.

#### 6. Suggested follow-ups

Keep the viewer-closed assertion aligned with `RTABMAPVIZ` when the vision stack rebases onto this line. The vision PRs also default the viewer to false; a line branch that sets it true will fail `b0d430e`’s test.

---

### `origin/feature/line-control`

#### 1. Summary

This integration branch contains `feature/px4-control` and then keeps going. Two later commits change the controller in ways PR #22 does not: `578017e` replaces the text wall file with a JSON geofence aligned to the spawn TF and EKF2, and `309ebe7` subscribes to `TwistStamped`. The rest of the commits that are not in `px4-control` are the sim, ground-truth, and CI fold-in (apt world, spawn.json, IMU source selection, ground-truth rate scaled with `/clock`).

#### 2. Stack position

- Tip: `d3873d4` `fix(ci): scale the ground-truth rate with /clock`.
- Contains `origin/feature/px4-control` (`8668a11` is an ancestor).
- Does not contain `flight-analysis` or `vision-modules`.
- Ahead of `main` by 81 commits. 32 commits are not in `line-base` or `px4-control`. Many of those 32 are the sim-headless history landing on this line (`2abc7f9` and earlier).

#### 3. Stats

The two control fixes, not the whole branch:

- `578017e`: 12 files, +719 / −58. `geofence.py` grows by about 308 lines. `test_geofence.py` grows by about 218. The example file changes from `walls_format_example.txt` to `walls_format_example.json`.
- `309ebe7`: 3 files, +63 / −45, almost all in `node.py`.

`d842511` is the merge-style commit that brings the TwistStamped tree onto the CI line; the behaviour fix is `309ebe7`.

#### 4. File-by-file commentary

**Geofence (`578017e`).** The new `geofence.py` header says the only wall format is `includes/gz/walls/<world>.json` with `"frame": "world_enu"` or `"local"`. `world_enu` segments stay inactive until a static `world` → `spawn` TF matches a landed local position whose EKF2 yaw is aligned. The rotation is the local heading minus the spawn heading, frozen at arming (`derive_frame`, `WorldLocalFrame`). A heading reset in flight brakes to a hold. Discovery timeouts use the steady clock; control `dt` stays on the node clock. `local` segments are used as-is.

This is the missing piece from PR #22’s plan (wall positions from the spawn transform, 90° spawn still listed as not tested there). It is real code on this branch, with a larger unit test, and it is not on `feat/px4-control`. PR #22’s text loader and empty-file behaviour are what you get if you merge #22 alone.

**TwistStamped (`309ebe7`).** `node.py` imports `TwistStamped`. `_on_cmd_vel` stores `msg.twist` and sets `_cmd_time` from `header.stamp`, not from the node clock. `note_cmd_vel`’s comment is updated to match. The same commit stops subscribing to the unversioned `vehicle_status` fallback (`_subscribe_px4(..., fallback=False)` for status only) because this PX4 publishes `vehicle_status_v1` only.

The stamp is the right clock when Nav2 uses sim time. A publisher that leaves the stamp at zero makes `command_age` look like the full sim time, and the executive brakes immediately. PR #22’s zero-twist ignore does not save you, because the stamp is already stale before the values are applied.

**Other unique commits.** `d3873d4` / `9b6843f` publish ground truth from the spawned x500 and scale the expected rate with `/clock`. `84fbc62` keeps vision rate gates out of the copied `.env`. `09e53ae` applies `sim.params` before SITL launch. Those belong to the sim/CI review, not to the vision-profile stack. `89c2d8e` and `1d5ed20` (“one cpu camera and one IMU”) are the integration’s answer to running vision on the CI line, and they are not the `VISION_PROFILE` implementation from PR #23.

#### 5. Review verdict

`line-control` is ahead of PR #22 on the two integration bugs that will show up as soon as Nav2 and a spawned world are in the loop: stamped velocity commands, and walls expressed in the world frame. The geofence commit is large enough to review on its own (freeze-at-arm, heading-reset brake, JSON schema). It should not be assumed tested just because PR #22’s smaller geofence tests exist.

Merge readiness: this is an integration branch, not a PR in the set you asked to merge. Cherry-pick `309ebe7` and `578017e` onto `feat/px4-control` before that PR is revived. Do not merge the 81-commit line to `main` to get those two fixes.

#### 6. Suggested follow-ups

1. Review `578017e` as its own change: the freeze conditions, what happens if yaw is aligned and the TF is wrong by 90°, and the steady-clock versus sim-clock split.
2. Reject a `TwistStamped` whose stamp is zero, and fall back to the node clock, so an unstamped command does not expire instantly.
3. Rebase those two commits onto current `feat/px4-control` rather than merging `line-control`.

---

### `origin/feature/line-sim`

#### 1. Summary

This is the sim and vision pipeline branch. It shares history with `line-base` (merge-base `89689a4`) and with `line-control` (merge-base `89c2d8e`). It does not contain `feature/vision-profiles`. Eight commits are not in `line-base` or `feature/sim-headless-groundtruth`. They fold apt_world, a single cpu camera and IMU, spawn.json, the closed RTAB-Map viewer, and a ground-truth publisher that follows the spawned x500 into the CI line.

#### 2. Stack position

- Tip: `c4aec60` `fix(ci): scale the ground-truth rate with /clock` (parallel message to `line-control`’s `d3873d4`, different hash).
- Ahead of `main` by 57. Contains `feature/sim-headless-groundtruth`. Does not contain `vision-profiles` or `px4-control`.

#### 3. Stats

Eight unique commits versus `line-base` plus `sim-headless-groundtruth`: `7dc3b4a`, `1d5ed20`, `da9ec1c`, `89c2d8e`, `6294c01`, `09aa676`, `7eff5dc`, `c4aec60`. A diff of `line-sim` against the `line-base` merge-base is on the order of 110 files and +33419 / −9391, because that range includes the sim-headless stack (worlds, URDF, tests), not only the eight commits.

#### 4. File-by-file commentary

The eight subjects, newest first:

- `c4aec60` scales the ground-truth rate gate with `/clock`, same intent as `line-control`.
- `7eff5dc` publishes ground truth from the spawned x500 rather than a fixed model name.
- `09aa676` applies `sim.params` before launch and expects the viewer to stay closed.
- `6294c01` marks the branch as the sim and vision pipeline branch.
- `89c2d8e` / `1d5ed20` keep one cpu camera and one IMU on the CI line.
- `da9ec1c` folds apt_world assets into that line.
- `7dc3b4a` keeps apt_world and default, and writes `spawn.json`.

`spawn.json` here is the producer PR #26’s plan says was missing. It lands on this sim line, not on `feat/flight-analysis`. The flight grader can consume a file this branch writes only after the two branches are combined.

The cpu-camera reduction is a CI resource choice. It is a different mechanism from PR #23’s `VISION_PROFILE=cpu`, which scales intrinsics and selects an ini. Merging both without looking will give you two ways to force a small camera.

#### 5. Review verdict

The unique commits are integration glue: one camera, params before launch, spawn.json, ground truth that follows the vehicle, and a sim-time rate gate. They are the right shape for a pipeline branch. They are not a substitute for reviewing PRs #23–#25, because the profile implementation is not here.

Merge readiness: do not merge to `main` as a vision PR. Use it as the sim side of a later rebase once `spawn.json`’s schema is the one `flight_analysis/spawn.py` parses (`px4_offset_s` null, spread, samples, xyz, yaw).

#### 6. Suggested follow-ups

1. Diff `spawn.json` as written by `7dc3b4a` against `flight_analysis/spawn.py` `_clock_from_file` and `_xyz_from_file`. The grader’s null-offset path is useless if the writer uses another key.
2. When the vision-profile stack rebases here, delete one of the two “cpu camera” mechanisms and keep `VISION_PROFILE`.

---

### `origin/feature/px4-imu-bridge-indoor-flight`

#### 1. Summary

Nine commits on `main` build an earlier indoor offboard path: a PX4 DDS IMU bridge, fixes to `includes/gz/microxrce_offboard.py`, Nav2 map remaps, a TwistStamped recovery publisher, sim-time forcing, and a short indoor-flight plan. The tip commit, `ef6b61c`, marks that plan superseded by `px4_control`. The branch does not contain PR #22, and PR #22 deletes the script this branch edits. The two trees conflict by construction.

#### 2. Stack position

- Tip: `ef6b61c` `docs: mark the indoor flight plan as superseded by px4_control`.
- Parent of the first commit is `main` (`5eb5fde`). Ahead 9, behind 0.
- Not an ancestor of `feature/px4-control`. Not contained in the line branches as a tip.

#### 3. Stats

Versus `main`: 26 files, +718 / −224. The large hunk is `includes/gz/microxrce_offboard.py` (+553 / −part of the old file in the combined stat). New file: `includes/gz/px4_imu_bridge.py` (93 lines). Also `.env.example`, both compose Dockerfiles, `gz_modifications.bash`, the RTAB-Map start scripts, `start_offboard_flight.sh`, and `docs/plans/px4_indoor_flight_closed-loop.md` (40 lines).

#### 4. File-by-file commentary

Commit order is the integration diary:

- `3a0fc56` adds the indoor closed-loop plan.
- `21156ce` adds `px4_imu_bridge.py` and the offboard flight blockers.
- `d43791a` makes the recovery `/cmd_vel` publisher `TwistStamped`. That is the same Nav2 type bug PR #22 reintroduces by subscribing to `Twist`, and the bug `309ebe7` fixes again on `line-control`.
- `3305668` points Nav2 costmaps and RViz at `/rtabmap/map`.
- `dccbb37` adjusts the Gazebo Classic offboard node for Humble and the offboard-loss failsafe.
- `be69e60` reworks the Harmonic offboard script: safer takeoff, position hold, VIO checks.
- `bbf78d1` forces sim time in the IMU bridge and simplifies world selection.
- `98df1ea` updates the FlightHeight example and adds a GCS profile to the indoor flight command.
- `ef6b61c` says the plan is superseded.

`px4_imu_bridge.py` is the piece PR #22 does not replace with an equivalent module. The control branch reads `vehicle_odometry` from PX4 and does not add a separate DDS IMU bridge. If a later flight still needs the bridge’s rate or frame fix, it has to be ported; merging this branch on top of #22 will fight over `microxrce_offboard.py`.

`gz_modifications.bash` on this branch still appends `param set` lines. PR #22 replaces that with `PX4_PARAM_*`. Taking both diffs requires the PR #22 version.

Checked-in `.pyc` files change size in the stat (`robot_state_publisher.launch.cpython-310.pyc`, and the 3.8 file goes to empty). Those do not belong in a flight fix.

#### 5. Review verdict

Treat this branch as historical. The useful residue is the TwistStamped lesson, the `/rtabmap/map` remap, and whatever `px4_imu_bridge.py` still does that `px4_control` does not. The offboard script edits are obsolete the moment #22 lands, and the branch’s own tip says the plan is superseded.

Merge readiness: do not merge. Cherry-pick only a specific fix that #22 and `line-control` still lack, and drop the `.pyc` hunks.

#### 6. Suggested follow-ups

1. Diff `px4_imu_bridge.py` against the odometry path in `px4_control/node.py`. Port the bridge only if a flight still depends on that topic.
2. Leave `microxrce_offboard.py` deleted. Do not resurrect it to keep this branch mergeable.
3. Confirm Nav2’s `/rtabmap/map` remap from `3305668` is already on the DevOps or sim line before you drop this branch.

---

### Cross-cutting

You have three unfinished products, not one stack.

The vision product is a clean stack: profiles, then goldens, then hygiene, then the module split. #24, #25, and #27 are safe to keep in that order. #23’s default change to `cpu` and the move of `full` to 1280×800 are the behaviour change, and the health numbers for that new `full` and for `hw` are still unmeasured. Run `includes/gz/oakd_s2/test_golden.py` on the #27 tip before you treat the split as behaviour-preserving.

The control product is PR #22 plus two fixes that already exist on `line-control` (`578017e`, `309ebe7`) and are absent from the PR branch. On the PR head, GPS read-back can throw, `NAV_DLL_ACT` 0 is not reliable against the x500 airframe, vision loss during takeoff does not land, and `/cmd_vel` is the wrong message type. The GPS flight table in the PR body is from an older commit. I would not merge #22 until those four are fixed and one vision cpu flight is logged.

The grader (PR #26) is a separate tool with synthetic tests and no real log. Its `spawn.json` reader is waiting on a writer that showed up later on `line-sim`, not on this branch. `e2e_limits.py` should not grow a third copy.

`line-base`, `line-sim`, and `line-control` are pipeline branches. They are ahead of `main` by 30, 57, and 81 commits because they carry DevOps and sim history. Review the unique commits named above. Do not review those branches as if they were the six PRs.

The IMU-bridge branch is superseded by its own last commit and conflicts with the deletion in #22.

---

## Document control

| Field | Value |
|---|---|
| Generated | 2026-11-12 13:12 Asia/Tehran |
| Reviewer model | Grok 4.7 medium (two parallel passes: sim/DevOps and control/vision) |
| Source | `gh pr view/diff` for #19–#29; `git log` / `git diff` on `origin/feature/*` and `origin/refactor/*` |
| Artifacts | `docs/reviews/pr-branch-review.md`, `docs/reviews/pr-branch-review.html`, Cursor canvas |
