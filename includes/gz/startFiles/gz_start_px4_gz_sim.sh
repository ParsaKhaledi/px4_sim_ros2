#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash

WORLD="${1:-default}"
MODEL="x500_depth"

cd "${HOME}/PX4-Autopilot" || exit 1

# Sim profile written by gz_modifications.bash. Stock v1.17 rcS reads
# PX4_PARAM_* before ekf2 start. Missing file: airframe defaults, and
# px4_control refuses to arm after the read-back fails.
PARAM_ENV="${HOME}/PX4-Autopilot/px4_control_params.env"
if [ -f "${PARAM_ENV}" ]; then
  set -a
  # shellcheck disable=SC1090
  . "${PARAM_ENV}"
  set +a
  echo "px4_control sim params loaded from ${PARAM_ENV}"
else
  echo "px4_control sim params missing (${PARAM_ENV}); PX4 keeps airframe defaults" >&2
fi

export PX4_GZ_MODEL_POSE="-3,-1.6,0,0,0,3.14"

if [ "${WORLD}" = "default" ] || [ -z "${WORLD}" ]; then
    make px4_sitl "gz_${MODEL}"
else
    export PX4_GZ_WORLD="${WORLD}"
    make px4_sitl "gz_${MODEL}_${WORLD}"
fi
