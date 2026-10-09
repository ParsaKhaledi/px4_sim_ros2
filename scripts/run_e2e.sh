#!/usr/bin/env bash
# Fly the out-and-back mission and restart PX4/Gazebo after a crash.
#
# One attempt runs inside the PX4 container. Exit 2 (or a killed process)
# is a crash: logs are snapshotted, the sim container is recreated, and
# the mission tries again until E2E_MAX_RETRIES restarts have been used.
# With E2E_CHECK_CAMERAS=1, blank EGL frames are a failed render, and the
# script tries the same mission once more on Xvfb.
#
# E2E_ATTEMPT_TIMEOUT (default 480s) bounds one attempt. E2E_TOTAL_TIMEOUT
# bounds the whole run. A timeout writes the reason to the step summary,
# collects logs, then the exit trap tears the project down.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/compose_cli.sh"

export HEADLESS=1
export RTABMAPVIZ=false
export World="${World:-default}"
export PX4_GZ_MODEL="${PX4_GZ_MODEL:-x500_depth}"
export E2E_GZ_MODEL="${E2E_GZ_MODEL:-${PX4_GZ_MODEL}}"
export E2E_MAX_RETRIES="${E2E_MAX_RETRIES:-2}"
export E2E_CRASH_TILT_DEG="${E2E_CRASH_TILT_DEG:-60}"
export E2E_CRASH_MIN_HEIGHT_M="${E2E_CRASH_MIN_HEIGHT_M:-0.20}"
export E2E_CRASH_IMPACT_SPEED_MPS="${E2E_CRASH_IMPACT_SPEED_MPS:-2.5}"
export E2E_CRASH_DIVERGENCE_M="${E2E_CRASH_DIVERGENCE_M:-1.5}"
export E2E_ODOM_TIMEOUT_S="${E2E_ODOM_TIMEOUT_S:-4}"
export E2E_LEG_TOLERANCE_M="${E2E_LEG_TOLERANCE_M:-0.03}"
export E2E_OVERSHOOT_M="${E2E_OVERSHOOT_M:-0.03}"
export E2E_SETTLE_TOLERANCE_M="${E2E_SETTLE_TOLERANCE_M:-0.02}"
export E2E_SETTLE_HOLD_S="${E2E_SETTLE_HOLD_S:-1}"
export E2E_SETTLE_TIMEOUT_S="${E2E_SETTLE_TIMEOUT_S:-4}"
export E2E_YAW_TOLERANCE_DEG="${E2E_YAW_TOLERANCE_DEG:-5}"
export E2E_YAW_SETTLE_DEG="${E2E_YAW_SETTLE_DEG:-3}"
export E2E_HOVER_HEIGHT_BAND_M="${E2E_HOVER_HEIGHT_BAND_M:-0.05}"
export E2E_HOVER_HEIGHT_HOLD_S="${E2E_HOVER_HEIGHT_HOLD_S:-2}"
export E2E_CHECK_CAMERAS="${E2E_CHECK_CAMERAS:-0}"
export E2E_WALL_SCALE="${E2E_WALL_SCALE:-1}"
export E2E_CAMERA_MIN_VARIANCE="${E2E_CAMERA_MIN_VARIANCE:-1.0}"
export E2E_ATTEMPT_TIMEOUT="${E2E_ATTEMPT_TIMEOUT:-480}"
export E2E_TOTAL_TIMEOUT="${E2E_TOTAL_TIMEOUT:-720}"
export GZ_HEADLESS_RENDERING="${GZ_HEADLESS_RENDERING:-0}"
export GZ_USE_XVFB="${GZ_USE_XVFB:-0}"
if [ "${GZ_USE_XVFB}" = "1" ]; then
  export DISPLAY="${DISPLAY:-:99}"
  export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml:compose.xvfb.yml}"
else
  export DISPLAY=""
  export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
fi

MAX_RETRIES="${E2E_MAX_RETRIES:-2}"
MAX_ATTEMPTS=$((MAX_RETRIES + 1))
if [ -n "${E2E_FLIGHT_DIR:-}" ]; then
  FLIGHTS="${E2E_FLIGHT_DIR}"
else
  FLIGHTS="${ROOT}/logs/flights"
fi
case "${FLIGHTS}" in
  "${ROOT}/logs/"*) ;;
  *)
    echo "E2E_FLIGHT_DIR must be inside ${ROOT}/logs so the container can write it." >&2
    exit 1
    ;;
esac
CONTAINER_FLIGHTS="/home/px4/volume/${FLIGHTS#"${ROOT}/"}"
REPORT="${FLIGHTS}/trajectory.json"
RUN_START="$(date +%s)"
BRINGUP_S=0
TEARDOWN_S=0
ATTEMPT_TIMES=""

note_summary() {
  local title="$1"
  local body="$2"
  echo "${title}: ${body}" >&2
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### ${title}"
      echo
      echo "${body}"
    } >> "${GITHUB_STEP_SUMMARY}"
  fi
}

cleanup() {
  status=$?
  local teardown_start
  teardown_start="$(date +%s)"
  if [ ! -f "${REPORT}" ]; then
    write_summary || true
  fi
  if [ "${SMOKE_KEEP_UP:-0}" != "1" ]; then
    "${ROOT}/scripts/compose_stack.sh" down || true
  fi
  TEARDOWN_S=$(( $(date +%s) - teardown_start ))
  print_timing || true
  exit "${status}"
}
trap cleanup EXIT

snapshot_attempt() {
  attempt="$1"
  dest="${FLIGHTS}/attempt-${attempt}"
  mkdir -p "${dest}/health"
  compose_setup
  "${COMPOSE[@]}" logs --no-color --tail 400 PX4 > "${dest}/container.log" 2>&1 || true
  if [ -d "${ROOT}/logs/health" ]; then
    cp -a "${ROOT}/logs/health/." "${dest}/health/" || true
  fi
  "${ROOT}/scripts/health_report.sh" > "${dest}/health_report.txt" || true
}

run_attempt() {
  attempt="$1"
  result="${CONTAINER_FLIGHTS}/attempt-${attempt}.json"
  compose_setup
  # ROS_DISTRO is expanded inside the container, not by this shell.
  # -e is a compose option, so it has to come before the service name.
  # After the service name, compose runs it as the container command.
  # shellcheck disable=SC2016
  timeout --foreground "${E2E_ATTEMPT_TIMEOUT}" \
    "${COMPOSE[@]}" exec -T \
    -e E2E_TAKEOFF_HEIGHT_M \
    -e E2E_HOVER_S \
    -e E2E_LEG_LENGTH_M \
    -e E2E_HOVER_DRIFT_M \
    -e E2E_LEG_TOLERANCE_M \
    -e E2E_OVERSHOOT_M \
    -e E2E_SETTLE_TOLERANCE_M \
    -e E2E_SETTLE_HOLD_S \
    -e E2E_SETTLE_TIMEOUT_S \
    -e E2E_YAW_TOLERANCE_DEG \
    -e E2E_YAW_SETTLE_DEG \
    -e E2E_HOVER_HEIGHT_BAND_M \
    -e E2E_HOVER_HEIGHT_HOLD_S \
    -e E2E_RETURN_TOLERANCE_M \
    -e E2E_HEIGHT_TOLERANCE_M \
    -e E2E_MAX_RETRIES \
    -e E2E_CRASH_TILT_DEG \
    -e E2E_CRASH_MIN_HEIGHT_M \
    -e E2E_CRASH_IMPACT_SPEED_MPS \
    -e E2E_CRASH_DIVERGENCE_M \
    -e E2E_ODOM_TIMEOUT_S \
    -e E2E_CHECK_CAMERAS \
    -e E2E_WALL_SCALE \
    -e E2E_CAMERA_MIN_VARIANCE \
    -e E2E_GZ_MODEL \
    -e E2E_ATTEMPT="${attempt}" \
    -e E2E_RESULT_PATH="${result}" \
    -e World \
    -e PX4_GZ_MODEL \
    PX4 \
    bash -lc '
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

check_distance_sensor() {
  local model="${PX4_GZ_MODEL}"
  local reason=""
  if [ "${model}" = "x500" ]; then
    reason="SKIP distance_sensor: plain x500 has no lidar."
  elif [[ "${model}" != *lidar* ]]; then
    reason="SKIP distance_sensor: ${model} has no downward lidar."
  fi
  if [ -n "${reason}" ]; then
    echo "${reason}"
    if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
      echo "${reason}" >> "${GITHUB_STEP_SUMMARY}"
    fi
    return 0
  fi
  compose_setup
  # shellcheck disable=SC2016
  topics="$(compose_exec PX4 bash -lc \
      'source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && ros2 topic list')"
  if ! grep -q '/fmu/out/distance_sensor' <<< "${topics}"; then
    echo "Expected /fmu/out/distance_sensor in ros2 topic list for ${model}." >&2
    return 1
  fi
  echo "distance_sensor topic is present for ${model}."
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

print_timing() {
  {
    echo "Timing: bring-up ${BRINGUP_S}s"
    echo "Timing: attempts ${ATTEMPT_TIMES:-none}"
    echo "Timing: teardown ${TEARDOWN_S}s"
  } | tee -a "${ROOT}/logs/timing.txt"
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### Timing"
      echo
      echo "- bring-up: ${BRINGUP_S}s"
      echo "- attempts: ${ATTEMPT_TIMES:-none}"
      echo "- teardown: ${TEARDOWN_S}s"
    } >> "${GITHUB_STEP_SUMMARY}"
  fi
}

fail_timeout() {
  local reason="$1"
  note_summary "Flight attempt timed out" "${reason}"
  snapshot_attempt "${attempt}" || true
  exit 124
}

within_total() {
  local elapsed=$(( $(date +%s) - RUN_START ))
  if [ "${elapsed}" -ge "${E2E_TOTAL_TIMEOUT}" ]; then
    fail_timeout "Flight run exceeded E2E_TOTAL_TIMEOUT (${E2E_TOTAL_TIMEOUT}s) after ${elapsed}s."
  fi
}

fly_until_done() {
  while true; do
    within_total
    if [ "${attempt}" -gt "${MAX_ATTEMPTS}" ]; then
      echo "Attempt cap reached (${MAX_ATTEMPTS})." >&2
      break
    fi
    echo "Flight attempt ${attempt}"
    mkdir -p "${FLIGHTS}/attempt-${attempt}"
    chmod 777 "${FLIGHTS}" "${FLIGHTS}/attempt-${attempt}" || true
    local attempt_start
    attempt_start="$(date +%s)"
    "${ROOT}/scripts/record_flight.sh" start "${FLIGHTS}/attempt-${attempt}" || true

    set +e
    run_attempt "${attempt}"
    code=$?
    set -e
    local attempt_s=$(( $(date +%s) - attempt_start ))
    ATTEMPT_TIMES="${ATTEMPT_TIMES} ${attempt}:${attempt_s}s"

    "${ROOT}/scripts/record_flight.sh" stop || true
    snapshot_attempt "${attempt}"
    if [ "${code}" -eq 124 ]; then
      fail_timeout "Flight attempt ${attempt} exceeded E2E_ATTEMPT_TIMEOUT (${E2E_ATTEMPT_TIMEOUT}s) after ${attempt_s}s."
    fi
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
      local restart_start
      restart_start="$(date +%s)"
      "${ROOT}/scripts/compose_stack.sh" recreate
      "${ROOT}/scripts/apply_px4_params.sh"
      BRINGUP_S=$((BRINGUP_S + $(date +%s) - restart_start))
      attempt=$((attempt + 1))
      continue
    fi
    break
  done
}

mkdir -p "${ROOT}/logs"
: > "${ROOT}/logs/timing.txt"
"${ROOT}/scripts/compose_stack.sh" down || true
# The PX4 user inside the container writes attempt-N.json here.
# The runner creates the directory, so it has to be world-writable.
mkdir -p "${FLIGHTS}"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/flights" "${FLIGHTS}" || true
bringup_start="$(date +%s)"
"${ROOT}/scripts/compose_stack.sh" up
BRINGUP_S=$(( $(date +%s) - bringup_start ))
"${ROOT}/scripts/apply_px4_params.sh"
"${ROOT}/scripts/assert_px4_params.sh"
check_distance_sensor

attempt=1
fly_until_done

if [ "${E2E_CHECK_CAMERAS}" = "1" ] && [ "${GZ_USE_XVFB}" != "1" ]; then
  if python3 "${ROOT}/tests/e2e/report.py" \
      --needs-render-fallback \
      --attempts-dir "${FLIGHTS}"; then
    echo "Camera frames were blank or missing under EGL. Restarting with Xvfb."
    export GZ_HEADLESS_RENDERING=0
    export GZ_USE_XVFB=1
    export DISPLAY=:99
    export COMPOSE_FILES="compose.yml:compose.ci.yml:compose.xvfb.yml"
    attempt=$((attempt + 1))
    restart_start="$(date +%s)"
    "${ROOT}/scripts/compose_stack.sh" recreate
    "${ROOT}/scripts/apply_px4_params.sh"
    BRINGUP_S=$((BRINGUP_S + $(date +%s) - restart_start))
    fly_until_done
  fi
fi

"${ROOT}/scripts/compose_stack.sh" logs || true
write_summary
