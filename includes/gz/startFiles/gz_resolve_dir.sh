# Resolve the includes/gz directory.
#
# Compose mounts this startFiles directory at /home/px4/volume/startFiles, so
# SCRIPT_DIR/.. is /home/px4/volume and does not contain scripts/. The tree
# itself is mounted at /home/px4/volume/includes/gz.
#
# SIM_GZ_DIR wins when it is set. Otherwise use SCRIPT_DIR/.. when that
# directory contains scripts/sim_origin.py, then the compose mount.
# SCRIPT_DIR must be set before calling gz_resolve_dir. Sets GZ_DIR.

gz_resolve_dir() {
  if [ -n "${SIM_GZ_DIR:-}" ]; then
    if [ -f "${SIM_GZ_DIR}/scripts/sim_origin.py" ]; then
      GZ_DIR="$(cd "${SIM_GZ_DIR}" && pwd)"
      return 0
    fi
    echo "ERROR: SIM_GZ_DIR=${SIM_GZ_DIR} has no scripts/sim_origin.py" >&2
    return 1
  fi
  if [ -f "${SCRIPT_DIR}/../scripts/sim_origin.py" ]; then
    GZ_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
    return 0
  fi
  if [ -f "/home/px4/volume/includes/gz/scripts/sim_origin.py" ]; then
    GZ_DIR="/home/px4/volume/includes/gz"
    return 0
  fi
  echo "ERROR: cannot find includes/gz (scripts/sim_origin.py)." >&2
  echo "ERROR: set SIM_GZ_DIR, or run from a checkout where startFiles sits inside includes/gz." >&2
  echo "ERROR: looked at ${SCRIPT_DIR}/.. and /home/px4/volume/includes/gz" >&2
  return 1
}
