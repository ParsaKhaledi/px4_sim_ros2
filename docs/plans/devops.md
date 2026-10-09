# DevOps

Branch: `feature/devops-compose-health-ci` at `b76daa5` (`fix(ci): wire image builds to the Actions cache`).

PRs #19 #20 #21 #22 #23 #26 closed when `feat/` was renamed to `feature/`. Cite branches, not those PRs. Nothing merges unless Parsa asks.

## Goal

Keep the base as small as possible: one image, one compose file (`docker-compose-px4.yml`), one PX4 parameter file applied before EKF2 starts, and one CI workflow (lint, unit tests, one headless flight).

The flight under test is `px4_control` on `feature/px4-control`. It is not `tests/e2e/px4_offboard.py` and not `microxrce_offboard.py` (`microxrce_offboard.py` is retired). The product goal is Nav2 closed-loop in `apt_world`. Until that stack exists, CI stays one headless hover/smoke and does not grow a second controller.

## Already on this branch

- Actions cache credentials and cache API v2 on image builds (`b76daa5`).
- `config/px4/params/sim.params` applied before EKF2, including clearing the SITL store and setting `PX4_PARAM_*` after the x500 airframe loads. `NAV_DLL_ACT 0` is already in that file.
- `docker-compose-px4.yml` passes the camera profile and the OAK-D mount; `RTABMAPVIZ` defaults to false.
- CI loads one image and lets the flight write its result. Jobs today: lint, colcon, image build, flight. A second GPU image job is still in the workflow.

## What stays

- One image, built in CI.
- `docker-compose-px4.yml`.
- One parameter file, `config/px4/params/sim.params`, applied before EKF2 starts.
- One workflow, `.github/workflows/ci.yml`: lint, unit tests, one headless hover/smoke.
- `.env.example` only. Do not commit `.env`.

## What is cut

- `nightly.yml`.
- Custom GitHub actions (`.github/actions/*`) and the reusable workflows (`.github/workflows/_build.yml`, `.github/workflows/_sim-test.yml`).
- Per-service HealthCheck YAML under `HealthCheck/checks/`.
- The observability branch, the image split (`docker-compose-px4-GPU.yml` and the second image), and the demo GIF.
- `tests/e2e/px4_offboard.py` and `microxrce_offboard.py` as the flight under test.
- `includes/gazebo_classic` is removed. That is a base cleanup; do not do the deletion in this plan.

## Ordered next steps

1. Collapse CI into `ci.yml` only: lint, unit tests, one headless hover/smoke. Remove `nightly.yml`, `_build.yml`, `_sim-test.yml`, `.github/actions/*`, and the GPU image job.
2. Keep `docker-compose-px4.yml` and one image. Drop the GPU compose and the second image.
3. Keep a single `sim.params`, applied before EKF2. Put only PX4 parameters there. Do not invent extra parameters.
   - `NAV_DLL_ACT 0` so headless can arm with no GCS.
   - EKF2 vision numbers, owned with `px4_control`: `EKF2_GPS_CTRL 0`, `EKF2_EV_CTRL 9`, `EKF2_HGT_REF 0` (baro), `EKF2_MAG_TYPE 5`, `EKF2_RNG_CTRL 1`.
   - The magnetometer is a Gazebo world plugin, owned by `feature/sim-headless-groundtruth`, not a line in `sim.params`.
   - `vehicle_status_v1` is the topic name `px4_control` subscribes to. It is not a parameter.
4. Point the one CI flight at that headless hover/smoke. Do not call `tests/e2e/px4_offboard.py` or `microxrce_offboard.py`, and do not add another controller. When `feature/px4-control` is the stack under test, that node is the flight; until Nav2 closed-loop in `apt_world` exists, CI stays the hover/smoke.
5. Stop tracking `.env`. Leave `.env.example`.
6. Leave `includes/gazebo_classic` in the tree. Note the removal; do not delete it here.

## Dependencies

- This base does not wait on sim, vision, or `px4_control`.
- The hover/smoke does not wait on Nav2.
- The five EKF2 values live in `sim.params` and are owned with `feature/px4-control`.
- Replacing the hover/smoke with `px4_control` waits on `feature/px4-control`.
- The Nav2 flight in `apt_world` waits on `feature/sim-headless-groundtruth` (vision folded in) and `feature/px4-control`.

Merge order, only when Parsa asks: this base, then `feature/sim-headless-groundtruth` (vision folded in), then `feature/px4-control`, then one passing Nav2 flight.

## Done when

- CI on `feature/devops-compose-health-ci` is one workflow and it is green: lint, unit tests, one headless hover/smoke.
- The tree this branch owns is one image, `docker-compose-px4.yml`, and one `sim.params` applied before EKF2, with `NAV_DLL_ACT 0` and the five EKF2 values above. The magnetometer plugin stays on the sim branch. `vehicle_status_v1` stays a `px4_control` subscription.
- Nightly, custom actions, per-service HealthCheck YAML, the GPU image split, the observability branch, and the demo GIF are out of this work.
- `.env` is not committed.
- The CI flight is not `px4_offboard.py` or `microxrce_offboard.py`, and there is still only one controller path.
