#!/bin/bash
set -eo pipefail
# Start PX4 SITL with Gazebo Harmonic.
#
# HEADLESS=1    no gz GUI and no X display. Cameras still render via EGL
#               (--headless-rendering). GUI mode is the default.
# HEADLESS_BACKEND=xvfb
#               use Xvfb instead of EGL when the binary is installed.
# HEADLESS_SOFTWARE=1
#               force Mesa software GL (llvmpipe). Useful on a VM with no GPU.
# PX4_GZ_MODEL_POSE
#               spawn pose "x,y,z,roll,pitch,yaw" in the Gazebo ENU world.
#               Default: -3,-1.6,0.15,0,0,3.14
#               z is drop clearance above the floor, not a ground height.

USER_NAME=px4
HOME=/home/${USER_NAME}

source /opt/ros/$ROS_DISTRO/setup.bash

WORLD="${1:-default}"
# x500 is the plain quad. x500_depth is the camera airframe.
# Flight CI sets PX4_GZ_MODEL=x500 so Gazebo does not have to render.
MODEL="${PX4_GZ_MODEL:-x500_depth}"
DEFAULT_POSE="-3,-1.6,0.15,0,0,3.14"

export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:--3,-1.6,0.15,0,0,3.14}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=gz_resolve_dir.sh
source "${SCRIPT_DIR}/gz_resolve_dir.sh"
gz_resolve_dir || exit 1
REPO_GZ="${GZ_DIR}"

# SIM_ORIGIN_* is the spawn coordinate. The script writes the matching
# world origin into PX4's worlds and exports it as PX4_HOME_LAT/LON/ALT.
# PX4 v1.17 gz uses those three variables as the world origin.
if ! origin_env="$(python3 "${REPO_GZ}/scripts/sim_origin.py")"; then
  echo "ERROR: sim_origin.py failed; GPS origin was not set" >&2
  exit 1
fi
eval "${origin_env}"

python3 "${REPO_GZ}/scripts/patch_x500_ground_truth.py" || \
  echo "WARN: could not patch x500_depth with the ground-truth plugin"
python3 "${REPO_GZ}/scripts/patch_x500_lidar_down.py" || \
  echo "WARN: could not patch x500_depth with the downward lidar"
if ! python3 "${REPO_GZ}/scripts/patch_x500_sensor_systems.py" "${WORLD}"; then
  echo "ERROR: launched world ${WORLD}.sdf was not found; sensor systems were not stripped" >&2
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="${REPO_GZ}/models:${GZ_SIM_RESOURCE_PATH:-}"

headless=0
case "${HEADLESS:-0}" in
  1|true|TRUE|yes|YES) headless=1 ;;
esac

if [ -z "${REAL_GZ:-}" ]; then
  REAL_GZ="$(command -v gz || true)"
  export REAL_GZ
fi
export PATH="${REPO_GZ}/bin:${PATH}"

if [ "${headless}" = "1" ]; then
  export HEADLESS=1
  backend="${HEADLESS_BACKEND:-egl}"
  if [ "${backend}" = "xvfb" ] && command -v Xvfb >/dev/null 2>&1; then
    export HEADLESS_BACKEND=xvfb
    export DISPLAY="${HEADLESS_DISPLAY:-:99}"
    display_num="${DISPLAY#:}"
    if [ ! -S "/tmp/.X11-unix/X${display_num}" ]; then
      Xvfb "${DISPLAY}" -screen 0 1280x1024x24 -ac +extension GLX +render -noreset \
        >/tmp/xvfb-gz.log 2>&1 &
      sleep 0.3
    fi
    echo "Gazebo headless backend: Xvfb on ${DISPLAY}"
  else
    if [ "${backend}" = "xvfb" ]; then
      echo "Xvfb is not installed; using EGL --headless-rendering"
    fi
    export HEADLESS_BACKEND=egl
    unset DISPLAY
    echo "Gazebo headless backend: EGL --headless-rendering (no GUI)"
  fi
  if [ "${HEADLESS_SOFTWARE:-0}" = "1" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    echo "Gazebo headless: software rendering enabled"
  fi
else
  echo "Gazebo GUI mode (set HEADLESS=1 to disable the GUI)"
fi

cd "${HOME}/PX4-Autopilot" || exit 1

# PX4 1.17 only generates gz_<model>_<world> for worlds that were present
# at cmake time. Custom worlds are copied in later, so that target is
# unknown. Build the firmware, apply sim.params before ekf2, then launch
# with PX4_GZ_WORLD.
python3 /home/px4/volume/includes/gz/patch_dds_topics.py \
    "${HOME}/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml"
make px4_sitl_default
python3 /home/px4/volume/scripts/px4_params.py apply \
    --params-dir /home/px4/volume/config/px4/params \
    --airframes "${HOME}/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes" \
    --rcs "${HOME}/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS" \
    --rootfs "${HOME}/PX4-Autopilot/build/px4_sitl_default/rootfs"
if [ -z "${WORLD}" ]; then
    WORLD=default
fi
export PX4_GZ_WORLD="${WORLD}"
export PX4_SIM_MODEL="gz_${MODEL}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
cd "${HOME}/PX4-Autopilot/build/px4_sitl_default/rootfs"
exec ../bin/px4
