#!/usr/bin/env bash
# Headless out-and-back. Skips cleanly when the Drone API is not installed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

export HEADLESS=1
export RTABMAPVIZ=false
export DISPLAY=""
export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"

cleanup() {
  status=$?
  "${ROOT}/scripts/record_flight.sh" stop "${ROOT}/logs/flights/e2e" || true
  "${ROOT}/scripts/compose_stack.sh" logs || true
  "${ROOT}/scripts/health_report.sh" || true
  if [ "${SMOKE_KEEP_UP:-0}" != "1" ]; then
    "${ROOT}/scripts/compose_stack.sh" down || true
  fi
  exit "${status}"
}
trap cleanup EXIT

"${ROOT}/scripts/compose_stack.sh" down || true
"${ROOT}/scripts/compose_stack.sh" up
"${ROOT}/scripts/record_flight.sh" start "${ROOT}/logs/flights/e2e"

docker exec \
  -e E2E_TAKEOFF_HEIGHT_M \
  -e E2E_HOVER_S \
  -e E2E_LEG_LENGTH_M \
  -e E2E_HOVER_DRIFT_M \
  -e E2E_LEG_TOLERANCE_M \
  -e E2E_YAW_TOLERANCE_DEG \
  -e E2E_YAW_SETTLE_DEG \
  -e E2E_RETURN_TOLERANCE_M \
  -e E2E_HEIGHT_TOLERANCE_M \
  -e E2E_REQUIRE_GROUND_TRUTH \
  px4_sim bash -lc \
  'source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && if [ -f /home/px4/volume/ros2_ws/install/setup.bash ]; then source /home/px4/volume/ros2_ws/install/setup.bash; fi && exec python3 /home/px4/volume/tests/e2e/test_out_and_back.py'
