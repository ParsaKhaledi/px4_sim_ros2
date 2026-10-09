#!/usr/bin/env bash
# Print the latest health JSONL row for each service and check.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${1:-${ROOT}/logs/health}"

python3 - "${LOG_DIR}" <<'PY'
import json
import sys
from pathlib import Path

log_dir = Path(sys.argv[1])
if not log_dir.is_dir():
    print(f"No health log directory at {log_dir}")
    sys.exit(0)

latest = {}
order = []
for path in sorted(log_dir.glob("*.jsonl")):
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        key = (row.get("service", path.stem), row.get("check", ""), row.get("target", ""))
        if key not in latest:
            order.append(key)
        latest[key] = row

if not latest:
    print(f"No health records in {log_dir}")
    sys.exit(0)

print(f"{'SERVICE':<16} {'CHECK':<14} {'TARGET':<36} {'STATUS':<6} {'RATE':>8}")
fails = 0
for key in order:
    row = latest[key]
    rate = row.get("rate_hz", "")
    rate_s = f"{rate:.1f}" if isinstance(rate, float) else str(rate)
    status = row.get("status", "")
    if status == "fail":
        fails += 1
    print(
        f"{str(row.get('service', '')):<16} "
        f"{str(row.get('check', '')):<14} "
        f"{str(row.get('target', '')):<36} "
        f"{status:<6} {rate_s:>8}"
    )
print(f"checks={len(latest)} failing={fails}")
sys.exit(0)
PY

if command -v docker >/dev/null 2>&1; then
  "${ROOT}/scripts/compose_stack.sh" ps || true
fi
