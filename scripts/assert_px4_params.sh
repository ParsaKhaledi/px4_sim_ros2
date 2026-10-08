#!/usr/bin/env bash
# Read the running SITL parameters and fail if the headless overrides missed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${PX4_CONTAINER:-px4_sim}"
BUILD_DIR="/home/px4/PX4-Autopilot/build/px4_sitl_default"

output=""
attempt=1
while [ "${attempt}" -le 8 ]; do
  if output="$(timeout 30 docker exec "${CONTAINER}" bash -lc "cd '${BUILD_DIR}' && ./bin/px4-param show")"; then
    if printf '%s\n' "${output}" | grep -q 'NAV_DLL_ACT'; then
      break
    fi
  fi
  echo "Waiting for the PX4 shell (${attempt}/8)."
  sleep 2
  attempt=$((attempt + 1))
done

printf '%s\n' "${output}" | grep -E 'NAV_DLL_ACT|NAV_RCL_ACT|COM_RC_IN_MODE|COM_RC_LOSS_T' || true
printf '%s\n' "${output}" | python3 "${ROOT}/scripts/check_px4_params.py"
