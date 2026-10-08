#!/usr/bin/env bash
# Set every PX4_PARAM_* from the running container after the airframe loads.
#
# rcS applies those variables before the airframe. When the value equals the
# firmware default, PX4 does not mark it changed, and the airframe
# `param set-default` restores its own value. `px4-param set` after the shell
# responds is an explicit change, including for 0.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/compose_cli.sh"
compose_setup

BUILD_DIR="/home/px4/PX4-Autopilot/build/px4_sitl_default"
ready=""
attempt=1
while [ "${attempt}" -le 8 ]; do
  if ready="$(timeout 30 compose_exec PX4 bash -lc "cd '${BUILD_DIR}' && ./bin/px4-param show")"; then
    if printf '%s\n' "${ready}" | grep -q 'NAV_DLL_ACT'; then
      break
    fi
  fi
  ready=""
  echo "Waiting for the PX4 shell before parameter set (${attempt}/8)."
  sleep 2
  attempt=$((attempt + 1))
done

if [ -z "${ready}" ] || ! printf '%s\n' "${ready}" | grep -q 'NAV_DLL_ACT'; then
  echo "PX4 shell did not respond before parameter set." >&2
  exit 1
fi

env_text="$(compose_exec PX4 bash -lc 'printenv')"
applied=0
while IFS= read -r command; do
  [ -n "${command}" ] || continue
  if ! [[ "${command}" =~ ^px4-param\ set\ [A-Z][A-Z0-9_]*\ [+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)$ ]]; then
    echo "Refusing unexpected parameter command: ${command}" >&2
    exit 1
  fi
  echo "PX4 params: ${command}"
  args="${command#px4-param }"
  timeout 30 compose_exec PX4 bash -lc "cd '${BUILD_DIR}' && ./bin/px4-param ${args}"
  applied=$((applied + 1))
done < <(printf '%s\n' "${env_text}" | python3 "${ROOT}/scripts/px4_params.py" commands)

if [ "${applied}" -eq 0 ]; then
  echo "PX4 params: no PX4_PARAM_* overrides in the container."
fi
