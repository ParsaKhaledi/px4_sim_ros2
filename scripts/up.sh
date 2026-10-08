#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [ ! -f .env ]; then
  echo "Missing .env file. Copy .env.example to .env and set px4TAG."
  exit 1
fi

# Caller exports win over .env, including the camera mount and the viewer flag.
_keep_if_set() {
  var_name=$1
  eval "_set=\${${var_name}+x}"
  eval "_val=\${${var_name}-}"
  eval "_saved_${var_name}_set=\${_set}"
  eval "_saved_${var_name}=\${_val}"
}
_restore_or_default() {
  var_name=$1
  default_value=$2
  eval "_set=\${_saved_${var_name}_set-}"
  if [ -n "${_set}" ]; then
    eval "${var_name}=\${_saved_${var_name}}"
  fi
  eval "export ${var_name}=\${${var_name}:-${default_value}}"
}
_restore_if_caller_set() {
  var_name=$1
  eval "_set=\${_saved_${var_name}_set-}"
  if [ -n "${_set}" ]; then
    eval "export ${var_name}=\${_saved_${var_name}}"
  fi
}
_keep_if_set CAM_PITCH_DEG
_keep_if_set CAM_X
_keep_if_set CAM_Y
_keep_if_set CAM_Z
_keep_if_set RTABMAPVIZ
_keep_if_set VISION_PROFILE
_keep_if_set CAM_RATE_HZ
_keep_if_set CAM_STEREO_WIDTH
_keep_if_set CAM_STEREO_HEIGHT
_keep_if_set CAM_COLOR_WIDTH
_keep_if_set CAM_COLOR_HEIGHT
_keep_if_set IMU_RATE_HZ

# shellcheck disable=SC1091
source .env

_restore_or_default CAM_PITCH_DEG 17
_restore_or_default CAM_X 0.12
_restore_or_default CAM_Y 0.03
_restore_or_default CAM_Z 0.242
_restore_or_default RTABMAPVIZ false
_restore_or_default VISION_PROFILE full
# Empty means "use the profile". Do not invent a number here.
_restore_if_caller_set CAM_RATE_HZ
_restore_if_caller_set CAM_STEREO_WIDTH
_restore_if_caller_set CAM_STEREO_HEIGHT
_restore_if_caller_set CAM_COLOR_WIDTH
_restore_if_caller_set CAM_COLOR_HEIGHT
_restore_if_caller_set IMU_RATE_HZ

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
