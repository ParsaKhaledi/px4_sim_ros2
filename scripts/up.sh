#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [ ! -f .env ]; then
  echo "Missing .env file. Copy .env.example to .env and set px4TAG."
  exit 1
fi

# shellcheck disable=SC1091
source .env

CameraType="${CameraType:-rgbd}"
World="${World:-default}"
COMPOSE_PROFILES="${COMPOSE_PROFILES:-gcs,slam,nav}"
COMPOSE_FILE="${COMPOSE_FILE:-docker-compose-px4.yml}"

if [ -n "${DISPLAY:-}" ]; then
  xhost +local: >/dev/null 2>&1 || true
fi

export CameraType World

PROFILE_ARGS=()
IFS=',' read -ra PROFILES <<< "${COMPOSE_PROFILES}"
for profile in "${PROFILES[@]}"; do
  profile="$(echo "${profile}" | xargs)"
  [ -n "${profile}" ] && PROFILE_ARGS+=(--profile "${profile}")
done

docker compose -f "${COMPOSE_FILE}" "${PROFILE_ARGS[@]}" up -d

echo "Stack started. Core services: PX4, StatePublisher"
echo "Profiles active: ${COMPOSE_PROFILES}"
echo "Optional tmux helpers: CameraType=${CameraType} World=${World} ./includes/gz/startFiles/tmux-session.sh"
