#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/load_env.sh"
load_repo_env "${ROOT}"

DOCKERFILE="${1:-dockerFile/Dockerfile_px4_sim_NO_GPU}"
IMAGE_TAG="${2:-${px4TAG}}"

BUILD_ARGS=(
  --build-arg "PX4_VERSION=${PX4_VERSION}"
  --build-arg "XRCE_AGENT_VERSION=${XRCE_AGENT_VERSION}"
  --build-arg "ROS_DISTRO=${ROS_DISTRO}"
  --build-arg "USER_UID=${USER_UID}"
  --build-arg "USER_GID=${USER_GID}"
)

if [[ "${DOCKERFILE}" == *GPU* ]]; then
  BUILD_ARGS+=(
    --build-arg "CUDA_ARCH_BIN=${CUDA_ARCH_BIN}"
    --build-arg "CUDA_ARCH_PTX=${CUDA_ARCH_PTX}"
    --build-arg "WITH_CUDA_OPENCV=${WITH_CUDA_OPENCV}"
    --build-arg "OPENCV_VERSION=${OPENCV_VERSION}"
    --build-arg "QGC_URL=${QGC_URL_GPU}"
  )
  IMAGE_TAG="${2:-${px4GPUTAG}}"
else
  BUILD_ARGS+=(--build-arg "QGC_URL=${QGC_URL_NO_GPU}")
fi

docker build \
  "${BUILD_ARGS[@]}" \
  -t "${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}" \
  -f "${DOCKERFILE}" \
  "${ROOT}"
