#!/usr/bin/env bash
# Bring the headless stack up or down. Used by the smoke and e2e scripts.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

ACTION="${1:-up}"

export HEADLESS="${HEADLESS:-1}"
export RTABMAPVIZ="${RTABMAPVIZ:-false}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export DISPLAY="${DISPLAY:-}"

COMPOSE_FILES="${COMPOSE_FILES:-compose.yml:compose.ci.yml}"
IFS=':' read -r -a FILE_ARR <<< "${COMPOSE_FILES}"
COMPOSE=(docker compose)
for file in "${FILE_ARR[@]}"; do
  COMPOSE+=(-f "${ROOT}/${file}")
done

PROFILE_ARGS=()
if [ -n "${COMPOSE_PROFILES:-}" ]; then
  IFS=',' read -ra PROFILES <<< "${COMPOSE_PROFILES}"
  for profile in "${PROFILES[@]}"; do
    profile="${profile// /}"
    if [ -n "${profile}" ]; then
      PROFILE_ARGS+=(--profile "${profile}")
    fi
  done
fi

mkdir -p "${ROOT}/logs/health" "${ROOT}/logs/flights"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/health" "${ROOT}/logs/flights" || true

cd "${ROOT}"
case "${ACTION}" in
  up)
    if ! docker image inspect "${PX4_IMAGE}" >/dev/null 2>&1; then
      docker pull "${PX4_IMAGE}"
    fi
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" up -d --wait --wait-timeout "${SMOKE_TEST_TIMEOUT:-900}"
    ;;
  down)
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" down --remove-orphans
    ;;
  logs)
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" logs --no-color > "${ROOT}/logs/smoke-containers.log" || true
    ;;
  ps)
    "${COMPOSE[@]}" "${PROFILE_ARGS[@]}" ps
    ;;
  *)
    echo "usage: $0 up|down|logs|ps" >&2
    exit 2
    ;;
esac
