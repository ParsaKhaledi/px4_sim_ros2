#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash

WORLD="${1:-default}"
MODEL="x500_depth"

cd "${HOME}/PX4-Autopilot" || exit 1

export PX4_GZ_MODEL_POSE="-3,-1.6,0,0,0,3.14"

# Custom worlds are copied in by gz_modifications.bash. Always use the base
# model target and select the world at runtime via PX4_GZ_WORLD — this works
# whether or not a world-specific cmake target was generated at image build.
if [ "${WORLD}" = "default" ] || [ -z "${WORLD}" ]; then
    make px4_sitl "gz_${MODEL}"
else
    export PX4_GZ_WORLD="${WORLD}"
    echo "Starting gz_${MODEL} with PX4_GZ_WORLD=${WORLD}"
    make px4_sitl "gz_${MODEL}"
fi
