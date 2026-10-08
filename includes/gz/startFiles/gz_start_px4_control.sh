#!/bin/bash
# Start the PX4 offboard control node.
# Expects px4_control and px4_control_interfaces to be built on top of px4_msgs v1.17.0.

set -euo pipefail

USER_NAME="${USER_NAME:-px4}"
HOME_DIR="${HOME_DIR:-/home/${USER_NAME}}"

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${HOME_DIR}/ws_px4/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME_DIR}/ws_px4/install/setup.bash"
fi
if [ -f "${HOME_DIR}/ws_control/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME_DIR}/ws_control/install/setup.bash"
fi

if ! ros2 pkg prefix px4_control >/dev/null 2>&1; then
  echo "px4_control is not installed. Build ros2_ws/src/px4_control and px4_control_interfaces against px4_msgs v1.17.0." >&2
  exit 1
fi

export ESTIMATION_MODE="${ESTIMATION_MODE:-vision}"
export PX4_MAX_YAW_RATE_DEG_S="${PX4_MAX_YAW_RATE_DEG_S:-30}"

WORLD_NAME="${World:-default}"
WALLS="${HOME_DIR}/volume/includes/gz/worlds/walls/${WORLD_NAME}.txt"
if [ -f "${WALLS}" ]; then
  export PX4_WALL_SEGMENTS_FILE="${WALLS}"
fi

exec ros2 launch px4_control px4_control.launch.py \
  use_sim_time:="${USE_SIM_TIME:-true}" \
  estimation_mode:="${ESTIMATION_MODE}" \
  max_yaw_rate_deg_s:="${PX4_MAX_YAW_RATE_DEG_S}"
