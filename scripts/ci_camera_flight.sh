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
# Estimation: vision-only EKF. GPS plugin and compass stay on the model for
# ground truth / sensors; fusion is off (EKF2_GPS_CTRL=0, EKF2_MAG_TYPE=5).
# Yaw and position come from RTAB-Map → /fmu/in/vehicle_visual_odometry with
# EKF2_EV_CTRL=15 (hpos, vpos, vel, yaw). EKF2_HGT_REF=0 (baro): EV vertical
# still aids via EV_CTRL bit 1, but Soft-GL VO height is not the EKF datum.
#
# Usage: ./scripts/ci_camera_flight.sh <rgbd|stereo> [PX4_IMAGE]
# Thin wrappers: ci_rgbd_flight.sh / ci_stereo_flight.sh
# CI path-filter kick: keep this script under scripts/** for PR runs.
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
# Xvfb GUI path: leave HEADLESS empty so PX4 starts `gz sim -g` on :99.
# HEADLESS=1 + Xvfb without a GUI leaves camera sensors blank on Soft-GL.
unset HEADLESS || true
export RTABMAPVIZ=false
export CameraType="${CAM}"
export World="${World:-default}"
export PX4_GZ_MODEL=x500_depth
export E2E_GZ_MODEL=x500_depth
export VISION_PROFILE=cpu
export COMPOSE_PROFILES=slam
export COMPOSE_SERVICES="PX4 StatePublisher Rtabmap"
# Soft-GL EGL publishes blank camera frames on Actions; Xvfb + gz GUI grades
# real images. compose.xvfb.yml stacks after compose.ci.yml.
export GZ_HEADLESS_RENDERING="${GZ_HEADLESS_RENDERING:-0}"
export GZ_USE_XVFB="${GZ_USE_XVFB:-1}"
export HEADLESS_BACKEND=xvfb
export HEADLESS_SOFTWARE=1
export DISPLAY="${DISPLAY:-:99}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export COMPOSE_FILES="${COMPOSE_FILES:-docker-compose-px4.yml:compose.ci.yml:compose.xvfb.yml}"
# Camera flight does not grade distance_sensor; skip gpu_lidar on CPU CI.
export LIDAR_DOWN="${LIDAR_DOWN:-0}"
export HEALTH_CAMERA_MIN_HZ="${HEALTH_CAMERA_MIN_HZ:-1}"
export HEALTH_SLAM_MIN_HZ="${HEALTH_SLAM_MIN_HZ:-1}"
export HEALTH_TIMING="${HEALTH_TIMING:-1}"
export CI_FLIGHT_TIMING="${CI_FLIGHT_TIMING:-1}"
export SMOKE_TEST_TIMEOUT="${SMOKE_TEST_TIMEOUT:-420}"

# Vision EKF: keep every x500 sensor. No SYS_HAS_MAG=0. GPS plugin stays;
# fusion off only via EKF2_GPS_CTRL=0. Compass present, unfused (MAG_TYPE=5).
export ESTIMATION_MODE=vision
export EKF2_EV_CTRL=15
export EKF2_EV_DELAY="${EKF2_EV_DELAY:-0}"
export PX4_PARAM_EKF2_EV_CTRL=15
export PX4_PARAM_EKF2_EV_DELAY="${EKF2_EV_DELAY}"
# HGT_REF=baro (0). Why: EV_CTRL=15 still fuses vertical EV as aiding, but
# Soft-GL RTAB-Map height is too noisy to be the EKF height reference.
unset PX4_PARAM_EKF2_HGT_REF || true
unset PX4_PARAM_EKF2_GPS_CTRL || true
unset PX4_PARAM_EKF2_MAG_TYPE || true
unset PX4_PARAM_SYS_HAS_MAG || true
unset PX4_PARAM_SYS_HAS_GPS || true
# Retry Ogre 1 for non-blank Soft-GL frames (prior CI: gz SIGABRT on ogre).
export PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE:-ogre}"

# Full out-and-back (hover + forward + return). Same pass criteria for both
# cameras via shared E2E_* defaults / .env.example. No Soft-GL band loosening.
unset E2E_HOVER_ONLY || true
export E2E_HEALTH_SERVICES="PX4 StatePublisher Rtabmap"
export E2E_TAKEOFF_HEIGHT_M="${E2E_TAKEOFF_HEIGHT_M:-1.5}"
export E2E_HOVER_S="${E2E_HOVER_S:-10}"
export E2E_LEG_LENGTH_M="${E2E_LEG_LENGTH_M:-0.3}"
export E2E_CHECK_CAMERAS="${E2E_CHECK_CAMERAS:-1}"
export E2E_REQUIRE_VISION_FUSION=1

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
# render_oakd.py must rewrite URDF/SDF as user px4 on the bind mount.
chmod -R a+rwX \
  "${ROOT}/includes/gz/models" \
  "${ROOT}/includes/gz/x500_tf_publisher" \
  "${ROOT}/includes/gz/worlds" \
  "${ROOT}/includes/gz/oakd_s2" 2>/dev/null || true

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "### ${CAM} CI flight (hover + out-and-back, vision EV)"
    echo
    echo "- CameraType=${CameraType}"
    echo "- PX4_GZ_MODEL=${PX4_GZ_MODEL}"
    echo "- VISION_PROFILE=${VISION_PROFILE}"
    echo "- ESTIMATION_MODE=${ESTIMATION_MODE}"
    echo "- EKF2_EV_CTRL=${EKF2_EV_CTRL} (hpos|vpos|vel|yaw)"
    echo "- EKF2_HGT_REF=0 (baro): EV vertical aids; Soft-GL VO height not the datum"
    echo "- EKF2_GPS_CTRL=0 EKF2_MAG_TYPE=5 (sensors stay; fusion off)"
    echo "- EKF2_EV_DELAY=${EKF2_EV_DELAY}"
    echo "- PX4_GZ_SIM_RENDER_ENGINE=${PX4_GZ_SIM_RENDER_ENGINE}"
    echo "- GZ_USE_XVFB=${GZ_USE_XVFB} HEADLESS_BACKEND=${HEADLESS_BACKEND}"
    echo "- COMPOSE_SERVICES=${COMPOSE_SERVICES}"
    echo "- E2E_TAKEOFF_HEIGHT_M=${E2E_TAKEOFF_HEIGHT_M} E2E_HOVER_S=${E2E_HOVER_S}"
    echo "- E2E_LEG_LENGTH_M=${E2E_LEG_LENGTH_M}"
    echo "- E2E_CHECK_CAMERAS=${E2E_CHECK_CAMERAS}"
    echo "- script=scripts/run_e2e.sh → tests/e2e/test_out_and_back.py"
  } >> "${GITHUB_STEP_SUMMARY}"
fi

echo "ci_camera_flight: CameraType=${CameraType} hover+out-and-back (estimation=${ESTIMATION_MODE} ev_ctrl=${EKF2_EV_CTRL} hgt_ref=baro render=${PX4_GZ_SIM_RENDER_ENGINE:-default} xvfb=${GZ_USE_XVFB} cameras=${E2E_CHECK_CAMERAS})"
exec "${ROOT}/scripts/run_e2e.sh" "${PX4_IMAGE}"
