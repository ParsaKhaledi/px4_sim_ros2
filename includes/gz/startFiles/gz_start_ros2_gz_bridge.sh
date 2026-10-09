#!/bin/bash
# Merge the ros_gz bridge config and start the sim helpers.
#
# USE_SIM_TIME (default true) is passed to the bridge, which publishes /clock.
# IMU_SOURCE=oak (default) appends config_gz_bridge_imu.yaml so gz /imu reaches
# ROS. IMU_SOURCE=px4 leaves /imu to px4_imu_relay.
# config_gz_bridge_sim.yaml adds /ground_truth/odom.

USER_NAME=px4
HOME=/home/${USER_NAME}
source /opt/ros/$ROS_DISTRO/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=gz_resolve_dir.sh
source "${SCRIPT_DIR}/gz_resolve_dir.sh"
gz_resolve_dir || exit 1
BASE_CONFIG="${GZ_DIR}/config_gz_bridge.yaml"
SIM_CONFIG="${GZ_DIR}/config_gz_bridge_sim.yaml"
IMU_CONFIG="${GZ_DIR}/config_gz_bridge_imu.yaml"
MERGED_CONFIG="/tmp/px4_gz_bridge.yaml"
IMU_SOURCE="${IMU_SOURCE:-oak}"

cat "${BASE_CONFIG}" > "${MERGED_CONFIG}"
printf '\n' >> "${MERGED_CONFIG}"
if [ -f "${SIM_CONFIG}" ]; then
  cat "${SIM_CONFIG}" >> "${MERGED_CONFIG}"
fi
case "${IMU_SOURCE}" in
  oak|OAK)
    if [ -f "${IMU_CONFIG}" ]; then
      printf '\n' >> "${MERGED_CONFIG}"
      cat "${IMU_CONFIG}" >> "${MERGED_CONFIG}"
    fi
    ;;
  px4|PX4)
    echo "IMU_SOURCE=px4: Gazebo /imu is not bridged"
    ;;
  *)
    echo "ERROR: IMU_SOURCE must be oak or px4, got ${IMU_SOURCE}" >&2
    exit 1
    ;;
esac

USE_SIM_TIME="${USE_SIM_TIME:-true}"

"${SCRIPT_DIR}/gz_start_sim_helpers.sh" &

ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="${MERGED_CONFIG}" \
  -p use_sim_time:="${USE_SIM_TIME}"
