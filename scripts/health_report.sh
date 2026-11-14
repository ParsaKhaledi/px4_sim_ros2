#!/usr/bin/env bash
# Print the latest health JSONL row for each service and check.
# Uses HealthCheck/healthcheck.py --report so the table shares the gate's
# auto_detect selection (one code path). Exits 1 when any row is failing.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${1:-${ROOT}/logs/health}"

python3 "${ROOT}/HealthCheck/healthcheck.py" --report "${LOG_DIR}"
status=$?

if command -v docker >/dev/null 2>&1; then
  "${ROOT}/scripts/compose_stack.sh" ps || true
fi

exit "${status}"
