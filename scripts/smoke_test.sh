#!/usr/bin/env bash
# Headless compose smoke test. Camera rates are checked before the rest.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/compose_cli.sh"

if [ -n "${1:-}" ]; then
  export PX4_IMAGE="$1"
fi

export HEADLESS=1
export RTABMAPVIZ=false
export DISPLAY=""
export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
export COMPOSE_PROFILES="${COMPOSE_PROFILES:-}"

cleanup() {
  status=$?
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
"${ROOT}/scripts/apply_px4_params.sh"
"${ROOT}/scripts/assert_px4_params.sh"
compose_setup

echo "Checking camera rates first."
# shellcheck disable=SC2016
compose_exec PX4 bash -lc \
  'source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && exec python3 /home/px4/volume/HealthCheck/healthcheck.py --service PX4 --group camera'

echo "Checking the rest of the PX4 graph."
# shellcheck disable=SC2016
compose_exec PX4 bash -lc \
  'source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && exec python3 /home/px4/volume/HealthCheck/healthcheck.py --service PX4'

echo "Smoke test passed."
