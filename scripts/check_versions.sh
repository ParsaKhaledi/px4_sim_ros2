#!/usr/bin/env bash
# Fail when Dockerfile ARG defaults drift from versions.env.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/versions.env"

fail=0

arg_default() {
  local file="$1"
  local name="$2"
  local raw
  raw="$(grep -E "^ARG ${name}=" "${file}" | head -1 | cut -d= -f2- || true)"
  raw="${raw%%#*}"
  raw="${raw%\"}"
  raw="${raw#\"}"
  raw="${raw%"${raw##*[![:space:]]}"}"
  printf '%s' "${raw}"
}

check_arg() {
  local file="$1"
  local name="$2"
  local expected="$3"
  local found
  found="$(arg_default "${file}" "${name}")"
  if [ "${found}" != "${expected}" ]; then
    echo "version mismatch: ${file} ARG ${name}=${found:-<missing>} expected ${expected}"
    fail=1
  fi
}

NO_GPU="${ROOT}/dockerFile/Dockerfile_px4_sim_NO_GPU"
GPU="${ROOT}/dockerFile/Dockerfile_px4_sim_with_GPU"

for file in "${NO_GPU}" "${GPU}"; do
  check_arg "${file}" PX4_VERSION "${PX4_VERSION}"
  check_arg "${file}" XRCE_AGENT_VERSION "${XRCE_AGENT_VERSION}"
  check_arg "${file}" ROS_DISTRO "${ROS_DISTRO}"
  check_arg "${file}" USER_UID "${USER_UID}"
  check_arg "${file}" USER_GID "${USER_GID}"
done

check_arg "${GPU}" CUDA_ARCH_BIN "${CUDA_ARCH_BIN}"
check_arg "${GPU}" CUDA_ARCH_PTX "${CUDA_ARCH_PTX}"
check_arg "${GPU}" WITH_CUDA_OPENCV "${WITH_CUDA_OPENCV}"
check_arg "${GPU}" OPENCV_VERSION "${OPENCV_VERSION}"
check_arg "${NO_GPU}" QGC_URL "${QGC_URL_NO_GPU}"
check_arg "${GPU}" QGC_URL "${QGC_URL_GPU}"

if [ "${PX4_MSGS_REF}" != "v${PX4_VERSION}" ]; then
  echo "PX4_MSGS_REF (${PX4_MSGS_REF}) must be v${PX4_VERSION} so px4_msgs matches the autopilot."
  fail=1
fi

for file in "${NO_GPU}" "${GPU}"; do
  # The Dockerfile keeps the version in a build-arg, so the pattern is literal.
  # shellcheck disable=SC2016
  if ! grep -q 'git clone --depth 1 -b v${PX4_VERSION} https://github.com/PX4/px4_msgs.git' "${file}"; then
    echo "px4_msgs clone in ${file} is not pinned to v\${PX4_VERSION}"
    fail=1
  fi
done

if [ "${fail}" -ne 0 ]; then
  exit 1
fi

echo "Version pins match versions.env (PX4 ${PX4_VERSION}, px4_msgs ${PX4_MSGS_REF}, XRCE ${XRCE_AGENT_VERSION}, ROS ${ROS_DISTRO})."
