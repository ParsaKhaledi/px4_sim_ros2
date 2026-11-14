#!/usr/bin/env bash
# CPU CI camera flight for CameraType=rgbd|stereo (OAK-D S2 on x500_depth).
#
# Shared by the CI flight matrix so RGB-D and stereo are graded the same way:
#   - bring up PX4 + StatePublisher + Rtabmap (slam profile)
#   - param readback + multi-service health (failing=0)
#   - takeoff → hover (~10 s at ~1.5 m) → forward → yaw 180° → return → land
#     via scripts/run_e2e.sh → tests/e2e/test_out_and_back.py (same script and
#     E2E_* pass criteria for both cameras; there is no separate stereo-only
#     flight script on this branch)
#
# Usage: ./scripts/ci_camera_flight.sh <rgbd|stereo> [PX4_IMAGE]
# Thin wrappers: ci_rgbd_flight.sh / ci_stereo_flight.sh
#
# Timing: logs/timing.txt always. HEALTH_TIMING=0 / CI_FLIGHT_TIMING=0 silence
# verbose phase lines.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

CAM="${1:-}"
case "${CAM}" in
  rgbd|stereo) ;;
  *)
    echo "usage: $0 rgbd|stereo [PX4_IMAGE]" >&2
    exit 2
    ;;
esac
shift || true
if [ -n "${1:-}" ]; then
  export PX4_IMAGE="$1"
fi

# --- Explicit camera / model (do not inherit compose defaults) ---
export HEADLESS=1
export RTABMAPVIZ=false
export DISPLAY="${DISPLAY:-}"
export CameraType="${CAM}"
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

# Full out-and-back (hover + forward + return). Same pass criteria for both
# cameras via shared E2E_* defaults / .env.example.
unset E2E_HOVER_ONLY || true
export E2E_HEALTH_SERVICES="PX4 StatePublisher Rtabmap"
export E2E_TAKEOFF_HEIGHT_M="${E2E_TAKEOFF_HEIGHT_M:-1.5}"
export E2E_HOVER_S="${E2E_HOVER_S:-10}"
export E2E_LEG_LENGTH_M="${E2E_LEG_LENGTH_M:-0.3}"
# Soft-GL RTF is often ~0.3–0.6; stretch wall waits for hover settle + legs.
export E2E_WALL_SCALE="${E2E_WALL_SCALE:-3}"
export E2E_CHECK_CAMERAS="${E2E_CHECK_CAMERAS:-1}"

_default_flight_dir="${ROOT}/logs/flights/${CAM}"
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

mkdir -p "${E2E_FLIGHT_DIR}"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/health" "${ROOT}/logs/flights" "${E2E_FLIGHT_DIR}" || true

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "### ${CAM} CI flight (hover + out-and-back)"
    echo
    echo "- CameraType=${CameraType}"
    echo "- PX4_GZ_MODEL=${PX4_GZ_MODEL}"
    echo "- VISION_PROFILE=${VISION_PROFILE}"
    echo "- COMPOSE_SERVICES=${COMPOSE_SERVICES}"
    echo "- E2E_TAKEOFF_HEIGHT_M=${E2E_TAKEOFF_HEIGHT_M} E2E_HOVER_S=${E2E_HOVER_S}"
    echo "- E2E_LEG_LENGTH_M=${E2E_LEG_LENGTH_M}"
    echo "- script=scripts/run_e2e.sh → tests/e2e/test_out_and_back.py"
  } >> "${GITHUB_STEP_SUMMARY}"
fi

echo "ci_camera_flight: CameraType=${CameraType} hover+out-and-back (health=${E2E_HEALTH_SERVICES})"
exec "${ROOT}/scripts/run_e2e.sh" "${PX4_IMAGE}"
