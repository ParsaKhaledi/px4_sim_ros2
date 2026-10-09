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

export HEADLESS="${HEADLESS:-1}"
export RTABMAPVIZ="${RTABMAPVIZ:-false}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export DISPLAY="${DISPLAY:-}"

compose_setup

mkdir -p "${ROOT}/logs/health" "${ROOT}/logs/flights"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/health" "${ROOT}/logs/flights" || true

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
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### Stack did not become healthy"
      echo
      echo "Timed out after ${timeout}s waiting for ${waited_for} to become healthy."
      echo
      echo '```'
      echo "${status:-no services reported}"
      echo '```'
    } >> "${GITHUB_STEP_SUMMARY}"
  fi
}

wait_up() {
  local timeout="${SMOKE_TEST_TIMEOUT:-900}"
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
