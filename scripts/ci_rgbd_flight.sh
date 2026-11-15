#!/usr/bin/env bash
# Thin wrapper: RGB-D CI camera flight (hover + out-and-back).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${ROOT}/scripts/ci_camera_flight.sh" rgbd "$@"
