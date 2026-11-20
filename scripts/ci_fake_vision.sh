#!/usr/bin/env bash
# Headless vision flight with a fake RTAB-Map feed. No camera rendering.
#
#   ./scripts/ci_fake_vision.sh clean
#   ./scripts/ci_fake_vision.sh dropout
#
# clean: takeoff, hover, 30 cm forward, yaw 180, 30 cm back, land.
# dropout: the feed stops mid-yaw. Graded on brake, hold, and land.
#
# EKF2 matches issue #38. GPS and mag plugins stay on the stock x500.
# NAV_DLL_ACT=0 is set here because this run is headless.
# The 8 cm / 12 cm / 10 deg limits are exported only for clean.
# Grader JSON: logs/flights/fake_vision/<mode>/grade.json
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

MODE="${1:-}"
case "${MODE}" in
  clean|dropout) ;;
  *)
    echo "usage: $0 clean|dropout [PX4_IMAGE]" >&2
    exit 2
    ;;
esac
shift || true
if [ -n "${1:-}" ]; then
  export PX4_IMAGE="$1"
fi

# Stock x500: GPS and mag plugins stay. No Oak-D, no RTAB-Map, no lidar.
export CameraType=none
export PX4_GZ_MODEL=x500
export E2E_GZ_MODEL=x500
export COMPOSE_SERVICES="PX4 StatePublisher"
export COMPOSE_PROFILES=
export LIDAR_DOWN=0
export RTABMAPVIZ=false
export GZ_USE_XVFB=0
export GZ_HEADLESS_RENDERING=0
export HEADLESS=1
export FAKE_VISION=1
export USE_SIM_TIME=true

# Issue #38. The installer reads EKF2_*; the host readback reads PX4_PARAM_*.
export ESTIMATION_MODE=vision
export EKF2_GPS_CTRL=0
export EKF2_MAG_TYPE=5
export EKF2_EV_CTRL=15
export EKF2_HGT_REF=0
export EKF2_EV_DELAY=0
export PX4_PARAM_EKF2_GPS_CTRL=0
export PX4_PARAM_EKF2_MAG_TYPE=5
export PX4_PARAM_EKF2_EV_CTRL=15
export PX4_PARAM_EKF2_HGT_REF=0
export PX4_PARAM_EKF2_EV_DELAY=0
export PX4_PARAM_NAV_DLL_ACT=0

# Offset from the world origin, yaw about 37 degrees (0.645772 rad).
export PX4_GZ_MODEL_POSE="-1.7,2.4,0.15,0,0,0.645772"

export FAKE_VISION_RATE_HZ=12.5
export FAKE_VISION_DELAY_MS=45
export FAKE_VISION_NOISE_M=0
export FAKE_VISION_NOISE_YAW_DEG=0
export FAKE_VISION_DRIFT_MPS=0
export FAKE_VISION_WORLD_AXES=1
export E2E_CHECK_CAMERAS=0
export E2E_HEALTH_SERVICES=""
export E2E_CHECK_VO_STAMPS=1
export E2E_VO_DELAY_MS=45
export E2E_VO_DELAY_TOLERANCE_MS=25
export E2E_LEG_LENGTH_M="${E2E_LEG_LENGTH_M:-0.3}"
unset E2E_HOVER_ONLY || true

apply_clean_limits() {
  # Only the clean run. .env.example stays at 3 cm and 5 deg.
  export E2E_LEG_TOLERANCE_M=0.08
  export E2E_RETURN_TOLERANCE_M=0.12
  export E2E_YAW_TOLERANCE_DEG=10
  export E2E_REQUIRE_YAW_PASS_90=1
  export E2E_REQUIRE_VISION_FUSION=1
  export E2E_GRADE_DROPOUT=0
  export FAKE_VISION_FAULT=none
}

if [ "${MODE}" = "clean" ]; then
  apply_clean_limits
else
  export E2E_GRADE_DROPOUT=1
  export E2E_REQUIRE_YAW_PASS_90=0
  export E2E_REQUIRE_VISION_FUSION=0
  export FAKE_VISION_FAULT=dropout
  export FAKE_VISION_DROPOUT_YAW_DEG=70
fi

export E2E_FLIGHT_DIR="${ROOT}/logs/flights/fake_vision/${MODE}"
mkdir -p "${E2E_FLIGHT_DIR}"
chmod 777 "${ROOT}/logs" "${ROOT}/logs/flights" "${E2E_FLIGHT_DIR}" || true

echo "ci_fake_vision: mode=${MODE} pose=${PX4_GZ_MODEL_POSE} ev_ctrl=${EKF2_EV_CTRL} delay=${FAKE_VISION_DELAY_MS}ms fault=${FAKE_VISION_FAULT}"

set +e
"${ROOT}/scripts/run_e2e.sh"
code=$?
set -e
if [ -f "${E2E_FLIGHT_DIR}/trajectory.json" ]; then
  cp "${E2E_FLIGHT_DIR}/trajectory.json" "${E2E_FLIGHT_DIR}/grade.json"
fi
python3 "${ROOT}/tests/e2e/height_summary.py" --flight-dir "${E2E_FLIGHT_DIR}" || echo "height summary failed" >&2
exit "${code}"
