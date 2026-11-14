#!/usr/bin/env bash
# Thin wrapper: stereo CI camera flight (hover + out-and-back).
# Same mission script and E2E_* pass criteria as RGB-D (ci_camera_flight.sh).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${ROOT}/scripts/ci_camera_flight.sh" stereo "$@"
