#!/usr/bin/env bash
# Read the running SITL parameters and fail if the merged files and env missed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/compose_cli.sh"
compose_setup
BUILD_DIR="/home/px4/PX4-Autopilot/build/px4_sitl_default"
SHOW="${ROOT}/logs/px4_param_show.txt"
mkdir -p "${ROOT}/logs"

output=""
attempt=1
while [ "${attempt}" -le 8 ]; do
  if output="$(timeout 30 compose_exec PX4 bash -lc "cd '${BUILD_DIR}' && ./bin/px4-param show")"; then
    if printf '%s\n' "${output}" | grep -q 'NAV_DLL_ACT'; then
      break
    fi
  fi
  echo "Waiting for the PX4 shell (${attempt}/8)."
  sleep 2
  attempt=$((attempt + 1))
done

printf '%s\n' "${output}" > "${SHOW}"
printf '%s\n' "${output}" | grep -E 'NAV_DLL_ACT|NAV_RCL_ACT|COM_RC_IN_MODE|COM_RC_LOSS_T' || true
printf '%s\n' "${output}" | python3 "${ROOT}/scripts/check_px4_params.py"

# Same capture, wrong expectation. No second boot. A gate that cannot
# fail would exit 0 here and fail this script.
set +e
printf '%s\n' "${output}" | python3 "${ROOT}/scripts/check_px4_params.py" --override NAV_DLL_ACT=99
wrong=$?
set -e
if [ "${wrong}" -eq 0 ]; then
  echo "Readback gate accepted a wrong NAV_DLL_ACT." >&2
  exit 1
fi
echo "Readback gate rejected a wrong NAV_DLL_ACT."
