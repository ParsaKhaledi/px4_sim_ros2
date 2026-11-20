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
export E2E_GRADE_DROPOUT="${E2E_GRADE_DROPOUT:-0}"
export E2E_REQUIRE_YAW_PASS_90="${E2E_REQUIRE_YAW_PASS_90:-0}"
export E2E_CHECK_VO_STAMPS="${E2E_CHECK_VO_STAMPS:-0}"
export E2E_VO_DELAY_MS="${E2E_VO_DELAY_MS:-}"
export E2E_VO_DELAY_TOLERANCE_MS="${E2E_VO_DELAY_TOLERANCE_MS:-25}"
export E2E_WALL_SCALE="${E2E_WALL_SCALE:-1}"
export E2E_CAMERA_MIN_VARIANCE="${E2E_CAMERA_MIN_VARIANCE:-1.0}"
export E2E_ATTEMPT_TIMEOUT="${E2E_ATTEMPT_TIMEOUT:-480}"
export E2E_TOTAL_TIMEOUT="${E2E_TOTAL_TIMEOUT:-720}"
export GZ_HEADLESS_RENDERING="${GZ_HEADLESS_RENDERING:-0}"
export GZ_USE_XVFB="${GZ_USE_XVFB:-0}"
_append_compose_file() {
  local file="$1"
  case ":${COMPOSE_FILES}:" in
    *:"${file}":*) ;;
    *) COMPOSE_FILES="${COMPOSE_FILES}:${file}" ;;
  esac
  export COMPOSE_FILES
}
if [ "${GZ_USE_XVFB}" = "1" ]; then
  # Keep HEADLESS empty: PX4 starts gz GUI on Xvfb for non-blank cameras.
  unset HEADLESS || true
  export DISPLAY="${DISPLAY:-:99}"
  export HEADLESS_BACKEND="${HEADLESS_BACKEND:-xvfb}"
  export HEADLESS_SOFTWARE="${HEADLESS_SOFTWARE:-1}"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
  _append_compose_file "compose.xvfb.yml"
else
  export HEADLESS=1
  export DISPLAY=""
  export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
fi

MAX_RETRIES="${E2E_MAX_RETRIES:-2}"
MAX_ATTEMPTS=$((MAX_RETRIES + 1))
if [ -n "${E2E_FLIGHT_DIR:-}" ]; then
  # CI often passes a repo-relative path (logs/flights/rgbd). Resolve it.
  case "${E2E_FLIGHT_DIR}" in
    /*) FLIGHTS="${E2E_FLIGHT_DIR}" ;;
    *) FLIGHTS="${ROOT}/${E2E_FLIGHT_DIR}" ;;
  esac
else
  FLIGHTS="${ROOT}/logs/flights"
fi
# Normalize when the parent dir already exists (resolve .. / duplicate slashes).
_flights_parent="$(dirname "${FLIGHTS}")"
_flights_base="$(basename "${FLIGHTS}")"
if [ -d "${_flights_parent}" ]; then
  FLIGHTS="$(cd "${_flights_parent}" && pwd)/${_flights_base}"
fi
case "${FLIGHTS}" in
  "${ROOT}/logs"|"${ROOT}/logs/"*) ;;
  *)
    echo "E2E_FLIGHT_DIR must be inside ${ROOT}/logs so the container can write it (got ${FLIGHTS})." >&2
    exit 1
    ;;
esac
export E2E_FLIGHT_DIR="${FLIGHTS}"
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
  "${COMPOSE[@]}" logs --no-color --tail 800 PX4 > "${dest}/container.log" 2>&1 || true
  # Ogre Soft-GL aborts: keep a focused crash slice for the report.
  {
    echo "PX4_GZ_SIM_RENDER_ENGINE=${PX4_GZ_SIM_RENDER_ENGINE:-}"
    echo "ESTIMATION_MODE=${ESTIMATION_MODE:-}"
    echo "EKF2_EV_CTRL=${EKF2_EV_CTRL:-}"
    echo "EKF2_EV_DELAY=${EKF2_EV_DELAY:-}"
    grep -E 'SIGABRT|Aborted|ogre|RenderEngine|segfault|Segmentation|gz sim' \
      "${dest}/container.log" | tail -80 || true
  } > "${dest}/render_engine_diag.txt" 2>&1 || true
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
    -e E2E_HOVER_ONLY \
    -e E2E_WALL_SCALE \
    -e E2E_CAMERA_MIN_VARIANCE \
    -e E2E_GZ_MODEL \
    -e E2E_REQUIRE_VISION_FUSION \
    -e E2E_GRADE_DROPOUT \
    -e E2E_REQUIRE_YAW_PASS_90 \
    -e E2E_CHECK_VO_STAMPS \
    -e E2E_VO_DELAY_MS \
    -e E2E_VO_DELAY_TOLERANCE_MS \
    -e E2E_ATTEMPT="${attempt}" \
    -e E2E_RESULT_PATH="${result}" \
    -e World \
    -e PX4_GZ_MODEL \
    -e CameraType \
    -e VISION_PROFILE \
    -e ESTIMATION_MODE \
    -e EKF2_EV_CTRL \
    -e EKF2_EV_DELAY \
    -e PX4_PARAM_EKF2_EV_CTRL \
    -e PX4_PARAM_EKF2_EV_DELAY \
    -e PX4_GZ_SIM_RENDER_ENGINE \
    -e HEALTH_CAMERA_MIN_HZ \
    -e HEALTH_SLAM_MIN_HZ \
    -e HEALTH_TIMING \
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

start_fake_rtabmap() {
  # Detached publisher. compose_exec uses -T and would wait on the node.
  # A crash recreate kills the previous process, so each attempt starts one.
  if [ "${FAKE_VISION:-0}" != "1" ]; then
    return 0
  fi
  export FAKE_VISION_DELAY_MS="${FAKE_VISION_DELAY_MS:-45}"
  export FAKE_VISION_RATE_HZ="${FAKE_VISION_RATE_HZ:-12.5}"
  export FAKE_VISION_NOISE_M="${FAKE_VISION_NOISE_M:-0}"
  export FAKE_VISION_NOISE_YAW_DEG="${FAKE_VISION_NOISE_YAW_DEG:-0}"
  export FAKE_VISION_DRIFT_MPS="${FAKE_VISION_DRIFT_MPS:-0}"
  export FAKE_VISION_FAULT="${FAKE_VISION_FAULT:-none}"
  export FAKE_VISION_EXPRESS_CAMERA="${FAKE_VISION_EXPRESS_CAMERA:-0}"
  export FAKE_VISION_WORLD_AXES="${FAKE_VISION_WORLD_AXES:-1}"
  export FAKE_VISION_DROPOUT_YAW_DEG="${FAKE_VISION_DROPOUT_YAW_DEG:-70}"
  export USE_SIM_TIME="${USE_SIM_TIME:-true}"
  export CAM_X="${CAM_X:-0.12}"
  export CAM_Y="${CAM_Y:-0.03}"
  export CAM_Z="${CAM_Z:-0.242}"
  export CAM_PITCH_DEG="${CAM_PITCH_DEG:-17}"
  compose_setup
  echo "Starting fake_rtabmap (fault=${FAKE_VISION_FAULT}, delay=${FAKE_VISION_DELAY_MS} ms)."
  "${COMPOSE[@]}" exec -d -T \
    -e FAKE_VISION_DELAY_MS \
    -e FAKE_VISION_RATE_HZ \
    -e FAKE_VISION_NOISE_M \
    -e FAKE_VISION_NOISE_YAW_DEG \
    -e FAKE_VISION_DRIFT_MPS \
    -e FAKE_VISION_FAULT \
    -e FAKE_VISION_EXPRESS_CAMERA \
    -e FAKE_VISION_WORLD_AXES \
    -e FAKE_VISION_DROPOUT_YAW_DEG \
    -e USE_SIM_TIME \
    -e CAM_X \
    -e CAM_Y \
    -e CAM_Z \
    -e CAM_PITCH_DEG \
    PX4 \
    bash -lc '
      set +u
      source /opt/ros/${ROS_DISTRO}/setup.bash
      source /home/px4/ws_px4/install/setup.bash
      if [ -f /home/px4/volume/ros2_ws/install/setup.bash ]; then
        source /home/px4/volume/ros2_ws/install/setup.bash
      fi
      export PYTHONPATH="/home/px4/volume/ros2_ws/src/px4_control${PYTHONPATH:+:$PYTHONPATH}"
      exec python3 -m px4_control.testing.fake_rtabmap
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
    start_fake_rtabmap
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
record_boot_order() {
  compose_setup
  local raw="${ROOT}/logs/px4-boot-tail.log"
  local out="${ROOT}/logs/px4_boot_order.txt"
  "${COMPOSE[@]}" logs --no-color --tail 20000 PX4 > "${raw}" 2>/dev/null || true
  python3 - "${raw}" "${out}" << 'PY'
import re
import sys
from pathlib import Path

raw = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
text = re.sub(r"\x1b\[[0-9;]*m", "", raw)
rows = []
for line in text.splitlines():
    if any(token in line for token in ("PX4 params:", "[ekf2]", "Cleared SITL", "Sourced airframe .post")):
        rows.append(line.strip())
Path(sys.argv[2]).write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")
if rows:
    print("\n".join(rows))
else:
    print("Boot order: no parameter or ekf2 lines in the captured log.")
PY
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### Boot order"
      echo
      echo '```'
      cat "${out}"
      echo '```'
    } >> "${GITHUB_STEP_SUMMARY}"
  fi
}

run_preflight_health() {
  # Optional multi-service health gate (RGB-D CI). E2E_HEALTH_SERVICES is a
  # space-separated list of healthcheck --service names. Final table must
  # report failing=0. Set E2E_HEALTH_SERVICES= to skip.
  local services="${E2E_HEALTH_SERVICES:-}"
  if [ -z "${services}" ]; then
    return 0
  fi
  local svc
  echo "Preflight health services: ${services} (CameraType=${CameraType:-unset})"
  for svc in ${services}; do
    echo "Health gate: ${svc}"
    # shellcheck disable=SC2016
    if [ "${svc}" = "PX4" ]; then
      compose_exec PX4 bash -lc \
        'source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && \
         python3 /home/px4/volume/HealthCheck/healthcheck.py --service PX4 --group camera && \
         exec python3 /home/px4/volume/HealthCheck/healthcheck.py --service PX4'
    else
      compose_exec PX4 bash -lc \
        "source /opt/ros/\${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && \
         exec python3 /home/px4/volume/HealthCheck/healthcheck.py --service ${svc}"
    fi
  done
  echo "Verifying final health table (failing=0)."
  "${ROOT}/scripts/health_report.sh"
}

bringup_start="$(date +%s)"
"${ROOT}/scripts/compose_stack.sh" up
BRINGUP_S=$(( $(date +%s) - bringup_start ))
record_boot_order
"${ROOT}/scripts/assert_px4_params.sh"
check_distance_sensor
run_preflight_health

attempt=1
fly_until_done

if [ "${E2E_CHECK_CAMERAS}" = "1" ] && [ "${GZ_USE_XVFB}" != "1" ]; then
  if python3 "${ROOT}/tests/e2e/report.py" \
      --needs-render-fallback \
      --attempts-dir "${FLIGHTS}"; then
    echo "Camera frames were blank or missing under EGL. Restarting with Xvfb GUI."
    export GZ_HEADLESS_RENDERING=0
    export GZ_USE_XVFB=1
    unset HEADLESS || true
    export DISPLAY=:99
    export HEADLESS_BACKEND=xvfb
    export HEADLESS_SOFTWARE=1
    export QT_QPA_PLATFORM=xcb
    # Keep docker-compose-px4.yml (or whatever CI already set); only add xvfb.
    export COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
    _append_compose_file "compose.xvfb.yml"
    attempt=$((attempt + 1))
    restart_start="$(date +%s)"
    "${ROOT}/scripts/compose_stack.sh" recreate
    BRINGUP_S=$((BRINGUP_S + $(date +%s) - restart_start))
    fly_until_done
  fi
fi

"${ROOT}/scripts/compose_stack.sh" logs || true
write_summary
