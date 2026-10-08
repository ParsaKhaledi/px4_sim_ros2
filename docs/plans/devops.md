# DevOps

## State at pause

Paused 03:17 Tehran, 9 Oct 2026. PR #19 is open from `feat/devops-compose-health-ci`. The latest pushed commit is `f6c06a3`. CI run [37861235535](https://github.com/ParsaKhaledi/px4_sim_ros2/actions/runs/37861235535) was still building the image and had no flight numbers.

## Where each piece of work was and where it stopped

- PR #19 plain x500 flight-test failure: on `feat/devops-compose-health-ci` (PR #19). `wip/devops-19-fix` was not created. Stopped after reading the failed job log of CI run [37851112017](https://github.com/ParsaKhaledi/px4_sim_ros2/actions/runs/37851112017): the attempt died with `PermissionError: [Errno 13] Permission denied: '/home/px4/volume/logs/flights/x500/attempt-1.json'` after readback showed `NAV_DLL_ACT=0`. The writable flight directory is in `f6c06a3`; that follow-up run had not finished.
- Queued #19 items (remove `container_name`, CI build-once, `distance_sensor`, camera rate, `VISION_PROFILE` defaults, parameter file, timeouts, folder README template): committed together in `f6c06a3` on `feat/devops-compose-health-ci` and pushed. Lint and colcon on run 37861235535 were green; the image build and both flights had not finished.
- UFO laptop test of #19: worktree `~/docker_ws/px4_test/pr19`, runner `run_pr19.sh`. Was pulling `alienkh/px4_sim:1.17.0_121` at ~02:50 Tehran. No flight results. Left running at pause.
- Overnight watch of UFO, CI, and agents: paused at 03:17 Tehran, 9 Oct 2026.
- `legacy/v3.0` snapshot: done. Commit `f1a1275` (`docs: mark legacy/v3.0 as a frozen snapshot`) and the README notice that the branch is frozen.
- `feat/observability`, cleanup batches, docs/demo GIF, image split: not started. Waiting for #19 to merge. Image split also waits for Parsa's go.
