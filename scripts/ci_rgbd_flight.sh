#!/usr/bin/env bash
# CPU CI flight for CameraType=rgbd (OAK-D S2 on x500_depth).
#
# Sets the RGB-D stack explicitly (never rely on compose defaults), then
# hands off to run_e2e.sh which:
#   - brings up PX4 + StatePublisher + Rtabmap (slam profile)
#   - runs the param readback gate
#   - runs PX4/StatePublisher/Rtabmap health (E2E_HEALTH_SERVICES) and
#     fails if the final table has failing>0
#   - flies takeoff → hover (~10 s at ~1.5 m) → land with crash retry
#     (E2E_HOVER_ONLY=1)
#
# Timing: logs/timing.txt always. HEALTH_TIMING / CI_FLIGHT_TIMING default
# on; set either to 0 to silence verbose phase lines.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

if [ -n "${1:-}" ]; then
  export PX4_IMAGE="$1"
fi

# --- Explicit camera / model (do not inherit compose defaults) ---
export HEADLESS=1
export RTABMAPVIZ=false
export DISPLAY="${DISPLAY:-}"
export CameraType=rgbd
export World="${World:-default}"
export PX4_GZ_MODEL=x500_depth
export E2E_GZ_MODEL=x500_depth
export VISION_PROFILE=cpu
export COMPOSE_PROFILES=slam
export COMPOSE_SERVICES="PX4 StatePublisher Rtabmap"
export COMPOSE_FILES="${COMPOSE_FILES:-docker-compose-px4.yml:compose.ci.yml}"
export GZ_HEADLESS_RENDERING="${GZ_HEADLESS_RENDERING:-1}"
export GZ_USE_XVFB="${GZ_USE_XVFB:-0}"
export HEALTH_CAMERA_MIN_HZ="${HEALTH_CAMERA_MIN_HZ:-1}"
export HEALTH_SLAM_MIN_HZ="${HEALTH_SLAM_MIN_HZ:-1}"
export HEALTH_TIMING="${HEALTH_TIMING:-1}"
export CI_FLIGHT_TIMING="${CI_FLIGHT_TIMING:-1}"
export SMOKE_TEST_TIMEOUT="${SMOKE_TEST_TIMEOUT:-420}"

# Hover-only mission + preflight health across all three services.
export E2E_HOVER_ONLY=1
export E2E_HEALTH_SERVICES="PX4 StatePublisher Rtabmap"
export E2E_TAKEOFF_HEIGHT_M="${E2E_TAKEOFF_HEIGHT_M:-1.5}"
export E2E_HOVER_S="${E2E_HOVER_S:-10}"
export E2E_CHECK_CAMERAS="${E2E_CHECK_CAMERAS:-1}"
# Absolute under the repo so run_e2e's bind-mount check accepts CI's relative
# `logs/flights/rgbd` value from the workflow env.
_default_flight_dir="${ROOT}/logs/flights/rgbd"
if [ -n "${E2E_FLIGHT_DIR:-}" ]; then
  case "${E2E_FLIGHT_DIR}" in
    /*) ;;
    *) E2E_FLIGHT_DIR="${ROOT}/${E2E_FLIGHT_DIR}" ;;
  esac
else
  E2E_FLIGHT_DIR="${_default_flight_dir}"
fi
export E2E_FLIGHT_DIR
export E2E_ATTEMPT_TIMEOUT="${E2E_ATTEMPT_TIMEOUT:-480}"
export E2E_TOTAL_TIMEOUT="${E2E_TOTAL_TIMEOUT:-900}"
export E2E_MAX_RETRIES="${E2E_MAX_RETRIES:-2}"

# Pass E2E_HOVER_ONLY into the container attempt.
# run_e2e already forwards many E2E_* vars; ensure the new ones are listed.
export E2E_HOVER_ONLY

mkdir -p "${E2E_FLIGHT_DIR}"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/health" "${ROOT}/logs/flights" "${E2E_FLIGHT_DIR}" || true

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "### RGB-D CI flight"
    echo
    echo "- CameraType=${CameraType}"
    echo "- PX4_GZ_MODEL=${PX4_GZ_MODEL}"
    echo "- VISION_PROFILE=${VISION_PROFILE}"
    echo "- COMPOSE_SERVICES=${COMPOSE_SERVICES}"
    echo "- E2E_TAKEOFF_HEIGHT_M=${E2E_TAKEOFF_HEIGHT_M} E2E_HOVER_S=${E2E_HOVER_S}"
    echo "- E2E_HOVER_ONLY=${E2E_HOVER_ONLY}"
  } >> "${GITHUB_STEP_SUMMARY}"
fi

echo "ci_rgbd_flight: starting run_e2e (hover-only, health services=${E2E_HEALTH_SERVICES})"
exec "${ROOT}/scripts/run_e2e.sh" "${PX4_IMAGE}"
