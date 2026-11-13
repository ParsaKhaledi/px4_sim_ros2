#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

if [ ! -f .env ]; then
  echo "Missing .env file. Copy .env.example to .env and set px4TAG."
  exit 1
fi

CameraType="${CameraType:-rgbd}"
World="${World:-default}"
COMPOSE_PROFILES="${COMPOSE_PROFILES:-gcs,slam,nav}"
COMPOSE_FILE="${COMPOSE_FILE:-docker-compose-px4.yml}"

if [ -n "${DISPLAY:-}" ] && [ "${HEADLESS:-0}" != "1" ]; then
  xhost +local: >/dev/null 2>&1 || true
fi

export CameraType World

PROFILE_ARGS=()
IFS=',' read -ra PROFILES <<< "${COMPOSE_PROFILES}"
for profile in "${PROFILES[@]}"; do
  profile="${profile#"${profile%%[![:space:]]*}"}"
  profile="${profile%"${profile##*[![:space:]]}"}"
  [ -n "${profile}" ] && PROFILE_ARGS+=(--profile "${profile}")
done

docker compose -f "${COMPOSE_FILE}" "${PROFILE_ARGS[@]}" up -d

echo "Stack started. Core services: PX4, StatePublisher"
echo "Profiles active: ${COMPOSE_PROFILES}"
echo "Image: ${PX4_IMAGE}"
echo "Spawn pose: ${PX4_GZ_MODEL_POSE}"
echo "Optional tmux helpers: CameraType=${CameraType} World=${World} ./includes/gz/startFiles/tmux-session.sh"
