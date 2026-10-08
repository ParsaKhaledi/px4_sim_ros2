#!/bin/bash
# Background helpers that run next to the ros_gz bridge:
#   /sim/real_time_factor, the spawn-frame static TF, /sim/preflight_check.

USER_NAME=px4
HOME=/home/${USER_NAME}

source /opt/ros/$ROS_DISTRO/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_SRC="$(cd "${SCRIPT_DIR}/../sim_ws/src" && pwd)"
export PYTHONPATH="${SIM_SRC}/sim_monitor:${PYTHONPATH:-}"

python3 -m sim_monitor.real_time_factor &
python3 -m sim_monitor.spawn_frame &
python3 -m sim_monitor.preflight_check &

wait
