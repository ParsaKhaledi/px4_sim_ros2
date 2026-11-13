#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash

WORLD="${1:-default}"
MODEL="x500_depth"

cd "${HOME}/PX4-Autopilot" || exit 1

# Sim profile from install_px4_control_params.bash (also called by
# gz_modifications.bash). Stock v1.17 rcS reads PX4_PARAM_* before ekf2
# start. Source the env file before make so the SITL process inherits it.
# The installer also writes airframe .post files so NAV_DLL_ACT survives
# the x500 set-default. Missing file: airframe defaults, and px4_control
# refuses to arm after the read-back fails.
PARAM_ENV="${HOME}/PX4-Autopilot/px4_control_params.env"
INSTALLER="${HOME}/volume/includes/gz/params/install_px4_control_params.bash"
if [ -f "${INSTALLER}" ]; then
  # shellcheck disable=SC1090
  . "${INSTALLER}"
  install_px4_control_params "${PARAM_ENV}" "${ESTIMATION_MODE:-vision}" || exit 1
fi
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

# Re-run the installer after make so build/ airframe .post files exist even
# when ROMFS was not yet present during gz_modifications.
if [ -f "${INSTALLER}" ]; then
  install_px4_control_params "${PARAM_ENV}" "${ESTIMATION_MODE:-vision}" || exit 1
fi

