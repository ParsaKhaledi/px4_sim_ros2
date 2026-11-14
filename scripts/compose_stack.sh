#!/usr/bin/env bash
# Bring the headless stack up or down. Used by the smoke and e2e scripts.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/compose_cli.sh"

ACTION="${1:-up}"

export RTABMAPVIZ="${RTABMAPVIZ:-false}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
# Xvfb camera CI: keep HEADLESS empty and use a real XCB platform so PX4
# starts gz GUI on the virtual display (blank sensors otherwise).
case "${GZ_USE_XVFB:-0}" in
  1|true|TRUE|yes|YES)
    unset HEADLESS || true
    export DISPLAY="${DISPLAY:-:99}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export HEADLESS_BACKEND="${HEADLESS_BACKEND:-xvfb}"
    export HEADLESS_SOFTWARE="${HEADLESS_SOFTWARE:-1}"
    ;;
  *)
    export HEADLESS="${HEADLESS:-1}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
    export DISPLAY="${DISPLAY:-}"
    ;;
esac

compose_setup

mkdir -p "${ROOT}/logs/health" "${ROOT}/logs/flights"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/health" "${ROOT}/logs/flights" || true
# CameraType=rgbd|stereo runs render_oakd.py which rewrites SDF/URDF under the
# includes/gz bind mount. The container user (px4) is not the GHA checkout owner.
case "${CameraType:-}" in
  rgbd|stereo|RGBD|Stereo)
    chmod -R a+rwX \
      "${ROOT}/includes/gz/models" \
      "${ROOT}/includes/gz/x500_tf_publisher" \
      "${ROOT}/includes/gz/worlds" \
      "${ROOT}/includes/gz/oakd_s2" 2>/dev/null || true
    ;;
esac

note_unhealthy() {
  local timeout="${SMOKE_TEST_TIMEOUT:-900}"
  local waited_for="${SERVICES[*]:-every service in this project}"
  echo "Timed out after ${timeout}s waiting for ${waited_for} to become healthy (healthcheck.py topic rates)." >&2
  local status
  status="$(compose_ps_status)"
  if [ -n "${status}" ]; then
    echo "${status}" >&2
  else
    echo "docker compose ps returned no services." >&2
  fi
  # Capture why a service exited before the EXIT trap tears the project down.
  mkdir -p "${ROOT}/logs"
  # Prefer sidecar logs; PX4's interactive pxh prompt floods the buffer.
  {
    for svc in StatePublisher Rtabmap NAV2 PX4; do
      echo "===== ${svc} ====="
      "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" logs --no-color --tail 80 "${svc}" 2>&1 || true
    done
  } > "${ROOT}/logs/smoke-containers.log" 2>&1 || true
  if [ -f "${ROOT}/logs/smoke-containers.log" ]; then
    echo "---- container logs (tail) ----" >&2
    tail -n 160 "${ROOT}/logs/smoke-containers.log" >&2 || true
  fi
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### Stack did not become healthy"
      echo
      echo "Timed out after ${timeout}s waiting for ${waited_for} to become healthy."
      echo
      echo '```'
      echo "${status:-no services reported}"
      echo '```'
      if [ -f "${ROOT}/logs/smoke-containers.log" ]; then
        echo
        echo "<details><summary>container logs (tail)</summary>"
        echo
        echo '```'
        tail -n 80 "${ROOT}/logs/smoke-containers.log" || true
        echo '```'
        echo
        echo "</details>"
      fi
    } >> "${GITHUB_STEP_SUMMARY}"
  fi
}

wait_up() {
  local timeout="${SMOKE_TEST_TIMEOUT:-900}"
  local svc
  # Bring PX4 up first so a crash is not hidden behind depends_on of
  # StatePublisher/Rtabmap, then start the rest with the same --wait budget.
  if [ "${#SERVICES[@]}" -gt 1 ]; then
    local has_px4=0
    local -a others=()
    for svc in "${SERVICES[@]}"; do
      if [ "${svc}" = "PX4" ]; then
        has_px4=1
      else
        others+=("${svc}")
      fi
    done
    if [ "${has_px4}" = "1" ]; then
      if ! "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" up -d --wait --wait-timeout "${timeout}" PX4; then
        note_unhealthy
        return 1
      fi
      if [ "${#others[@]}" -gt 0 ]; then
        if ! "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" up -d --wait --wait-timeout "${timeout}" "${others[@]}"; then
          note_unhealthy
          return 1
        fi
      fi
      return 0
    fi
  fi
  if ! "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" up -d --wait --wait-timeout "${timeout}" "${SERVICES[@]}"; then
    note_unhealthy
    return 1
  fi
}

cd "${ROOT}"
case "${ACTION}" in
  up)
    if ! docker image inspect "${PX4_IMAGE}" >/dev/null 2>&1; then
      docker pull "${PX4_IMAGE}"
    fi
    wait_up
    ;;
  down)
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" down --remove-orphans
    ;;
  recreate)
    # Stop and recreate the sim containers. A Gazebo world reset leaves
    # PX4's estimator in the crashed state, so the container has to go.
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" stop "${SERVICES[@]}" || true
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" rm -f "${SERVICES[@]}" || true
    wait_up
    ;;
  logs)
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" logs --no-color > "${ROOT}/logs/smoke-containers.log" || true
    ;;
  ps)
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" ps
    ;;
  *)
    echo "usage: $0 up|down|recreate|logs|ps" >&2
    exit 2
    ;;
esac
