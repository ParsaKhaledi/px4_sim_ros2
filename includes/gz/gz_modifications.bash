#!/bin/bash
# Overlay camera models, or leave the stock PX4 model for a flight-only start.

echo "Start Modifications for running Simulation"
input=$1
WORKDIR=/home/px4
# PX4 parameters (offboard failsafe + EKF2). Rewrites one env file.
# gz_start_px4_gz_sim.sh sources it. Stock rcS applies PX4_PARAM_* before ekf2.
# ESTIMATION_MODE=vision|gps (default vision). See docs/px4_control.md.
# shellcheck disable=SC1091
. "$WORKDIR/volume/includes/gz/params/install_px4_control_params.bash"
install_px4_control_params \
  "$WORKDIR/PX4-Autopilot/px4_control_params.env" \
  "${ESTIMATION_MODE:-vision}"
# Camera Modifications:
echo "Selected Camera Type: $input"
# Render SDF and URDF from CAM_PITCH_DEG / CAM_X / CAM_Y / CAM_Z before the
# stereo-or-rgbd folder swap, so Gazebo and TF see the same mount.
GZ_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [ "$input" = none ] || [ "$input" = flight ]; then
    echo "Flight-only start. Stock PX4 model, no camera overlay."
elif [ "$input" = Stereo ] || [ "$input" = stereo ]; then
    python3 "${GZ_DIR}/oakd_s2/render_oakd.py" || exit 1
    rm -rf $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-stereo $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacements with $input camera is done"
elif [ "$input" = rgbd ] || [ "$input" = RGBD ] ; then
    python3 "${GZ_DIR}/oakd_s2/render_oakd.py" || exit 1
    rm -rf $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-rgbd $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacements with $input camera is done"
else
    echo "Invalid input, please try again."
    exit 1
fi

# PX4's x500_depth include is what actually places the camera in Gazebo.
# The URDF camera_joint was rendered above to the same pose.
if [ "$input" = Stereo ] || [ "$input" = stereo ] || [ "$input" = rgbd ] || [ "$input" = RGBD ]; then
    X500_SDF="$WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf"
    if [ -f "$X500_SDF" ]; then
        python3 "${GZ_DIR}/oakd_s2/render_oakd.py" --patch-x500 "$X500_SDF" || exit 1
    else
        echo "x500_depth model not found at $X500_SDF; URDF mount was still rendered"
    fi
fi

# Software-rendered CI flights pass GZ_CAMERA_UPDATE_RATE. Rewrite the copy
# under the PX4 tree only. The Oak-D files in this repo stay unchanged.
if [ -n "${GZ_CAMERA_UPDATE_RATE:-}" ] && [ -f "$WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf" ]; then
    echo "Software camera overlay: ${GZ_CAMERA_UPDATE_RATE} Hz. Image size stays with the model."
    python3 "$WORKDIR/volume/includes/gz/tune_gz_cameras.py" \
        --model "$WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf" \
        --rate "${GZ_CAMERA_UPDATE_RATE}"
fi
