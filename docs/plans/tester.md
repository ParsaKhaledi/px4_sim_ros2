# Tester

## Goal
Keep testing simple. One pytest entry, two levels: `unit` and `flight`. CI on `feature/devops-compose-health-ci` grades the flight. The only flight grader is `tests/e2e` on that branch. The controller under test is `px4_control` on `feature/px4-control`. Later, Nav2 drives it with `/cmd_vel` (`TwistStamped`).

## What I check
- `unit`: the fast tests, through that one pytest entry. No simulator.
- `flight`: the same entry, run by CI on `feature/devops-compose-health-ci`. Until Nav2 closed loop exists, that flight is the one headless hover. It uses `px4_control` once that branch is the stack under test, and it does not grow a second controller.
- When Nav2 is in the loop, the same flight level grades `/cmd_vel` as `TwistStamped` into `px4_control`. That replaces the hover. It is not a second CI job.

## What I stopped doing
- A fresh clone and a hand review of every branch.
- An overnight watch.
- A separate UFO queue.
- A `sim` level. The levels are `unit` and `flight`.
- Tests of `microxrce_offboard.py`. That script is retired.
- A second grader. `feature/flight-analysis` stays parked until a real `.ulg` exists.

## Next steps
1. Grade the one CI hover on `feature/devops-compose-health-ci` with `tests/e2e`.
2. Make `unit` and `flight` the two levels of one pytest entry. Run `unit` first, then `flight`. Stop at the first failure.
3. Confirm that flight drives `px4_control` from `feature/px4-control`.
4. Leave `feature/flight-analysis` unused until a real `.ulg` exists.
5. When Nav2 publishes `/cmd_vel` as `TwistStamped`, grade that path with the same flight level and the same grader.
6. Report the grade. Merge only when Parsa asks.

## Done when
- One pytest command runs `unit` and `flight`.
- The flight grade is the CI run on `feature/devops-compose-health-ci`, scored only by `tests/e2e`.
- That flight uses `px4_control`.
- `feature/flight-analysis` is still parked, with no second grader.
- Nothing is merged unless Parsa asks.
