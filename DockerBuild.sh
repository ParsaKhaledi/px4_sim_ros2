#!/bin/bash
set -euo pipefail

DOCKERFILE="${1:-dockerFile/Dockerfile_px4_sim_NO_GPU}"
IMAGE_TAG="${2:-local}"

docker build \
  --build-arg PX4_VERSION="${PX4_VERSION:-1.17.0}" \
  --build-arg XRCE_AGENT_VERSION="${XRCE_AGENT_VERSION:-v2.4.2}" \
  -t "alienkh/px4_sim:${IMAGE_TAG}" \
  -f "${DOCKERFILE}" \
  .
