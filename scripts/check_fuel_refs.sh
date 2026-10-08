#!/usr/bin/env bash
# Report Gazebo Fuel URIs. Warn-only unless FUEL_CHECK_STRICT=1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mapfile -t hits < <(grep -R -n -I "fuel.gazebosim.org" \
  "${ROOT}/includes/gz" "${ROOT}/includes/gazebo_classic" \
  --include='*.sdf' --include='*.urdf' --include='*.xacro' --include='*.world' \
  2>/dev/null || true)

if [ "${#hits[@]}" -eq 0 ]; then
  echo "No fuel.gazebosim.org references under includes/."
  exit 0
fi

echo "Found ${#hits[@]} fuel.gazebosim.org references."
printf '%s\n' "${hits[@]}" | head -n 40
echo
if [ "${FUEL_CHECK_STRICT:-0}" = "1" ]; then
  echo "FUEL_CHECK_STRICT=1, failing."
  exit 1
fi

echo "Warn-only: shipped worlds still reference fuel.gazebosim.org."
echo "Set FUEL_CHECK_STRICT=1 once those meshes are vendored."
exit 0
