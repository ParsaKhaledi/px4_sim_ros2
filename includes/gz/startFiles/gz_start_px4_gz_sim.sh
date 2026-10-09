#!/bin/bash
set -eo pipefail

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash

WORLD="${1:-default}"
# x500 is the plain quad. x500_depth is the camera airframe.
# Flight CI sets PX4_GZ_MODEL=x500 so Gazebo does not have to render.
MODEL="${PX4_GZ_MODEL:-x500_depth}"

cd "${HOME}/PX4-Autopilot" || exit 1

export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:--3,-1.6,0,0,0,3.14}"

# Camera models need a render even when HEADLESS=1. PX4 then starts
# `gz sim -s`, which has no GUI. Mesa llvmpipe supplies the GL driver.
# GZ_HEADLESS_RENDERING=1 adds Gazebo's EGL flag. GZ_USE_XVFB=1 is the
# fallback when those frames come out blank.
if [ "${GZ_USE_XVFB:-0}" = "1" ]; then
    export DISPLAY="${DISPLAY:-:99}"
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    display_number="${DISPLAY#:}"
    if [ ! -S "/tmp/.X11-unix/X${display_number}" ]; then
        Xvfb "${DISPLAY}" -screen 0 1280x1024x24 >/tmp/xvfb.log 2>&1 &
        waited=0
        while [ ! -S "/tmp/.X11-unix/X${display_number}" ] && [ "${waited}" -lt 20 ]; do
            sleep 0.25
            waited=$((waited + 1))
        done
    fi
elif [ "${GZ_HEADLESS_RENDERING:-0}" = "1" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    export EGL_PLATFORM=surfaceless
    mkdir -p /tmp/gz-bin
    cat > /tmp/gz-bin/gz << 'EOF'
#!/bin/sh
# PX4 launches `gz sim -s`. Add EGL rendering for camera sensors.
if [ "$1" = "sim" ]; then
    shift
    query=0
    for arg in "$@"; do
        case "$arg" in
            --versions|--help|-h|-g) query=1 ;;
        esac
    done
    if [ "$query" = "1" ]; then
        exec /usr/bin/gz sim "$@"
    fi
    exec /usr/bin/gz sim --headless-rendering "$@"
fi
exec /usr/bin/gz "$@"
EOF
    chmod +x /tmp/gz-bin/gz
    export PATH="/tmp/gz-bin:${PATH}"
fi

# PX4 1.17 only generates gz_<model>_<world> for worlds that were present
# at cmake time. Custom worlds are copied in later, so that target is
# unknown. Build the firmware, then launch with PX4_GZ_WORLD.
python3 /home/px4/volume/includes/gz/patch_dds_topics.py \
    "${HOME}/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml"
make px4_sitl_default
python3 /home/px4/volume/scripts/px4_params.py apply \
    --params-dir /home/px4/volume/config/px4/params \
    --airframes "${HOME}/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes" \
    --rootfs "${HOME}/PX4-Autopilot/build/px4_sitl_default/rootfs"
if [ -z "${WORLD}" ]; then
    WORLD=default
fi
export PX4_GZ_WORLD="${WORLD}"
export PX4_SIM_MODEL="gz_${MODEL}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
cd "${HOME}/PX4-Autopilot/build/px4_sitl_default/rootfs"
exec ../bin/px4
