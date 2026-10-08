#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
source /opt/ros/$ROS_DISTRO/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=gz_resolve_dir.sh
source "${SCRIPT_DIR}/gz_resolve_dir.sh"
gz_resolve_dir || exit 1
BASE_CONFIG="${GZ_DIR}/config_gz_bridge.yaml"
SIM_CONFIG="${GZ_DIR}/config_gz_bridge_sim.yaml"
MERGED_CONFIG="/tmp/px4_gz_bridge.yaml"

cat "${BASE_CONFIG}" > "${MERGED_CONFIG}"
printf '\n' >> "${MERGED_CONFIG}"
if [ -f "${SIM_CONFIG}" ]; then
  cat "${SIM_CONFIG}" >> "${MERGED_CONFIG}"
fi

USE_SIM_TIME="${USE_SIM_TIME:-true}"

"${SCRIPT_DIR}/gz_start_sim_helpers.sh" &

ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="${MERGED_CONFIG}" \
  -p use_sim_time:="${USE_SIM_TIME}"
