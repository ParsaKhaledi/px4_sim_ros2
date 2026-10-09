# Software and DevOps Tester: plan and status

Stopped on Oct 9, 2026 at 03:20 Tehran at Parsa's request (weekly usage at 98%). Work resumes only when Parsa says so.

## My role
- Check each team member's PR from a fresh clone: the tests, a clean tree after the tests, authorship, CI results, and whether each claimed fix is really in the code.
- Grade the flight-test numbers against the `E2E_*` limits in `.env`.
- Check that crashes and slowdowns show up in `run_summary.json`.
- Clean up the test suite: one entry point with `unit`, `sim` and `flight` levels, each with a hard time limit and stopping at the first failure.

## Where each piece of work was and where it stopped
| Item | Where | State when stopped |
|---|---|---|
| #19 CI check (`feat/devops-compose-health-ci`) | GitHub Actions run 37851112017 on `d80f229` | Lint, colcon and build passed. The build took 36 min, then flight_test rebuilt the image (about 35 min) and the plain x500 flight failed at about 02:47. DevOps had started finding the cause. Not graded yet. |
| #21 check (`feat/sim-headless-groundtruth`) | fresh clone on the shared computer | `bfd299d` passed: 63 tests, ground-truth leak fix confirmed in the code, Parsa as author. `d8d8d03` (66 tests, gitignore, preflight fixes) not checked yet. |
| #23 check (`feat/vision-profiles`) | fresh clone | `d5734d3` passed code review: 48 tests, B1 fix confirmed, `cpu` is the default. Two commits have a Cursor co-author trailer, which waits on Parsa confirming the git rule in the group chat. No live `cpu` run yet. |
| #26 check (`feat/flight-analysis`) | not started | Ready at `f1cd670` (42 tests). Agreed in review: limits come only from the environment and `.env`; spawn pose and clock offset come from `spawn.json`. |
| #20, #22, #24, #25, #27 checks | not started | #20 has never run CI. #22 is now at `7e7887f`. Its fixes for Flight Mechanics' five bugs need regression tests that fail on `019a4a5` and pass on the fix. |
| Overnight check | Grok Bot routine | Paused at 03:20 before its first run. |
| Evaluation log | `/workspace/tester_evals/log.md` on the shared computer | Has the #19, #21 and #23 results. |
| Live runs on UFO | Parsa's laptop | Not started. I was last in the queue (DevOps, Sim, Vision, PX4, Tester). |
| Test-suite cleanup | not started | Waits for #19 to merge. |

## Next steps when work resumes
1. Grade #19's next CI run: the cause of the x500 failure, both flights' numbers against the `.env` limits, crash-reset behaviour and the parameter-file checks. Flag any job that goes past its time limit.
2. Check from a fresh clone, in this order: #21 `d8d8d03`, #26 `f1cd670`, #22 `7e7887f` (regression tests for the five bugs, vision-mode parameters, `GF_ACTION=5`), then Vision's #24, #25 and #27 (snapshots unchanged).
3. Check the documentation rules everyone agreed on:
   - every code folder has a README;
   - every env setting matches between `.env` and the docs;
   - the ruff docstring rule fails only on changed lines until the comment pass is done;
   - the demo GIF stays under 8 MB at `docs/media/demo.gif` and comes from a flight that passed.
4. On UFO, when it's my turn: parameter-file negative tests, crash, slowdown and vision-off injections checked against `run_summary.json`, overhead with `TIMING_LOG=0`, and a comparison with the baseline at `~/docker_ws/px4_test/baseline_20261009T020250.txt`.
5. Test-suite cleanup after #19 merges.
6. Check every PR's authorship against the git rule Parsa confirms in the group chat.
