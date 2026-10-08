# Vision plan (Image Processing Researcher)

Owner: Image Processing Researcher. Scope: the OAK-D stereo/RGB-D camera, the RTAB-Map visual odometry, and vision logging and timing.
Status as of 2026-10-09 03:20 (Tehran). All work is stopped because of the usage limit and resumes only when Parsa says so.

## Where everything is and where it stopped

| Item | Where | State / where it stopped |
|---|---|---|
| #20 OAK-D S2 vision | `feat/oakd-s2-vision` @ `8a27573` | Done and unmerged (42 tests). Waiting on the #19 merge, then a rebase onto #19. |
| Vision profiles `cpu`/`full`/`hw` | `feat/vision-profiles` @ `d5734d3` | Done and unmerged. The `full`/`hw` (1280x800) floors are unmeasured, since GPU runs are off until Parsa says otherwise. |
| #24 golden snapshot tests | `refactor/vision-golden-tests` @ `de0a33a` (draft) | Done. 102 tests pass. Waiting for Parsa's review. |
| #25 cleanup A: hygiene | `refactor/vision-hygiene` @ `d78960e` (draft) | Done. Dead scripts removed, constants named, docstrings added, shellcheck findings down from 12 to 3. Waiting for review. |
| #27 cleanup B: modules | `refactor/vision-modules` @ `5dcf589` (draft) | Done. `geometry.py` was split into `env/hardware/profiles/gpu/intrinsics/frames/sdf/urdf/x500_patch`, and the golden snapshots are unchanged. Waiting for review. |
| PR C: health log, relay, GPU probe, vision READMEs | Not started | Planned in `/workspace/vision_cleanup/plan.md` (F31, F1, F12, F13, F17, F39, F40, F7, F38, F42, F43). Starts after Parsa reviews #24, #25 and #27. |
| Live checks on UFO (cpu stereo and RGB-D, 20 s hover scale check on #21, `LIDAR_DOWN` 1 vs 0 rates, cpu inlier margin) | UFO, my queue slot after Simulation | Not started. Every command needs Parsa's approval, with a 10-minute cap per run. |
| Stereo scale check on the shared box | `/workspace/px4_scale_check/` | The CPU-profile rate results are in `/workspace/px4_cpu_vision_run/`. The 7% scale question is still open. |
| Demo GIF (world + x500 + RTAB-Map view) | Planned: `docs/media/demo.gif` via `make demo-gif` | Not started. I own the RViz config (map cloud, path over ground truth, features on the left image). Simulation owns the capture script. |
| Per-folder READMEs and comments for vision code | Template in #19 (`docs/templates/FOLDER_README.md`) | Docstrings are done in #25/#27. The READMEs are in PR C. My lines for the `startFiles` README were handed to Simulation. |
| Vision timing and observability | Branch from `feat/observability` | Not started. Waiting on DevOps' helper. |
| Follow-ups | Not started | The URDF `base_link` 0.24 m offset, vision delay from the `.ulg`, removing `microxrce_offboard.py`, sim-time rates and stale-topic checks in `vision_rate_probe.py` and `check_vision_pipeline.py`. |

## Order when work resumes
1. Parsa reviews #24 -> #25 -> #27, then merges them when he asks.
2. PR C, including the vision READMEs.
3. After #19 and #20 merge: rebase #20, take generated files out of git, a single launch script, one compose/env source (settle `.env` with Tester), docs and meshes.
4. UFO live checks in my queue slot, then the demo GIF RViz config and a check that the GIF shows healthy tracking.
5. Vision timing on `feat/observability`.
