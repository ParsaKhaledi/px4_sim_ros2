#!/usr/bin/env bash
# Fly the out-and-back mission and restart PX4/Gazebo after a crash.
#
# One attempt runs inside the PX4 container. Exit 2 (or a killed process)
# is a crash: logs are snapshotted, the sim container is recreated, and
# the mission tries again until E2E_MAX_RETRIES restarts have been used.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

export HEADLESS=1
export RTABMAPVIZ=false
export DISPLAY=""
export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
export World="${World:-default}"
export PX4_GZ_MODEL="${PX4_GZ_MODEL:-x500_depth}"
export E2E_GZ_MODEL="${E2E_GZ_MODEL:-${PX4_GZ_MODEL}}"
export E2E_MAX_RETRIES="${E2E_MAX_RETRIES:-2}"
export E2E_CRASH_TILT_DEG="${E2E_CRASH_TILT_DEG:-60}"
export E2E_CRASH_MIN_HEIGHT_M="${E2E_CRASH_MIN_HEIGHT_M:-0.20}"
export E2E_CRASH_IMPACT_SPEED_MPS="${E2E_CRASH_IMPACT_SPEED_MPS:-2.5}"
export E2E_CRASH_DIVERGENCE_M="${E2E_CRASH_DIVERGENCE_M:-1.5}"
export E2E_ODOM_TIMEOUT_S="${E2E_ODOM_TIMEOUT_S:-4}"

MAX_RETRIES="${E2E_MAX_RETRIES:-2}"
CONTAINER="${PX4_CONTAINER:-px4_sim}"
FLIGHTS="${ROOT}/logs/flights"
REPORT="${FLIGHTS}/trajectory.json"

cleanup() {
  status=$?
  if [ ! -f "${REPORT}" ]; then
    write_summary || true
  fi
  if [ "${SMOKE_KEEP_UP:-0}" != "1" ]; then
    "${ROOT}/scripts/compose_stack.sh" down || true
  fi
  exit "${status}"
}
trap cleanup EXIT

snapshot_attempt() {
  attempt="$1"
  dest="${FLIGHTS}/attempt-${attempt}"
  mkdir -p "${dest}/health"
  docker logs --tail 400 "${CONTAINER}" > "${dest}/container.log" 2>&1 || true
  if [ -d "${ROOT}/logs/health" ]; then
    cp -a "${ROOT}/logs/health/." "${dest}/health/" || true
  fi
  "${ROOT}/scripts/health_report.sh" > "${dest}/health_report.txt" || true
}

run_attempt() {
  attempt="$1"
  result="/home/px4/volume/logs/flights/attempt-${attempt}.json"
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
    -e E2E_MAX_RETRIES \
    -e E2E_CRASH_TILT_DEG \
    -e E2E_CRASH_MIN_HEIGHT_M \
    -e E2E_CRASH_IMPACT_SPEED_MPS \
    -e E2E_CRASH_DIVERGENCE_M \
    -e E2E_ODOM_TIMEOUT_S \
    -e E2E_GZ_MODEL \
    -e E2E_ATTEMPT="${attempt}" \
    -e E2E_RESULT_PATH="${result}" \
    -e World \
    -e PX4_GZ_MODEL \
    "${CONTAINER}" bash -lc '
      set -eo pipefail
      set +u
      source /opt/ros/${ROS_DISTRO}/setup.bash
      source /home/px4/ws_px4/install/setup.bash
      if [ -f /home/px4/volume/ros2_ws/install/setup.bash ]; then
        source /home/px4/volume/ros2_ws/install/setup.bash
      fi
      exec python3 /home/px4/volume/tests/e2e/test_out_and_back.py
    '
}

write_summary() {
  mkdir -p "${FLIGHTS}"
  set +e
  python3 "${ROOT}/tests/e2e/report.py" \
    --attempts-dir "${FLIGHTS}" \
    --out "${REPORT}"
  summary_code=$?
  set -e
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### Flight"
      echo
      echo '```json'
      head -n 120 "${REPORT}" || true
      echo '```'
    } >> "${GITHUB_STEP_SUMMARY}"
  fi
  return "${summary_code}"
}

"${ROOT}/scripts/compose_stack.sh" down || true
"${ROOT}/scripts/compose_stack.sh" up

attempt=1
while true; do
  echo "Flight attempt ${attempt} of $((MAX_RETRIES + 1))"
  mkdir -p "${FLIGHTS}/attempt-${attempt}"
  "${ROOT}/scripts/record_flight.sh" start "${FLIGHTS}/attempt-${attempt}" || true

  set +e
  run_attempt "${attempt}"
  code=$?
  set -e

  "${ROOT}/scripts/record_flight.sh" stop || true
  snapshot_attempt "${attempt}"
  host_json="${FLIGHTS}/attempt-${attempt}.json"
  if [ ! -f "${host_json}" ]; then
    python3 "${ROOT}/tests/e2e/report.py" \
      --stub \
      --exit-code "${code}" \
      --attempt "${attempt}" \
      --out "${host_json}"
  fi

  if python3 "${ROOT}/tests/e2e/report.py" \
      --should-retry \
      --exit-code "${code}" \
      --attempt "${attempt}" \
      --max-retries "${MAX_RETRIES}"; then
    echo "Attempt ${attempt} crashed (exit ${code}). Restarting PX4 and Gazebo."
    "${ROOT}/scripts/compose_stack.sh" recreate
    attempt=$((attempt + 1))
    continue
  fi
  break
done

"${ROOT}/scripts/compose_stack.sh" logs || true
write_summary
