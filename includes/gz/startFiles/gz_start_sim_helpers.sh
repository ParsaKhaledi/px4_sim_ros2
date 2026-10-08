#!/bin/bash
# Background helpers that run next to the ros_gz bridge:
#   /sim/real_time_factor, the spawn frame, the camera static TF,
#   /sim/preflight_check, and px4_imu_relay when IMU_SOURCE=px4.

USER_NAME=px4
HOME=/home/${USER_NAME}

source /opt/ros/$ROS_DISTRO/setup.bash

# px4_msgs lives in the image workspace. Without it, get_message() throws
# on vehicle_status and the preflight timer kills the node.
PX4_MSGS_SETUP="${PX4_MSGS_WS_SETUP:-$HOME/ws_px4/install/setup.bash}"
if [ -f "${PX4_MSGS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${PX4_MSGS_SETUP}"
else
  echo "WARN: px4_msgs workspace not found at ${PX4_MSGS_SETUP}" >&2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=gz_resolve_dir.sh
source "${SCRIPT_DIR}/gz_resolve_dir.sh"
gz_resolve_dir || exit 1
SIM_SRC="${GZ_DIR}/sim_ws/src"
export PYTHONPATH="${SIM_SRC}/sim_monitor:${PYTHONPATH:-}"

python3 -m sim_monitor.real_time_factor &
python3 -m sim_monitor.spawn_frame &
python3 -m sim_monitor.camera_tf &
case "${IMU_SOURCE:-oak}" in
  oak|OAK) ;;
  px4|PX4)
    python3 -m sim_monitor.px4_imu_relay &
    ;;
  *)
    echo "ERROR: IMU_SOURCE must be oak or px4, got ${IMU_SOURCE}" >&2
    exit 1
    ;;
esac
python3 -m sim_monitor.preflight_check &

wait
