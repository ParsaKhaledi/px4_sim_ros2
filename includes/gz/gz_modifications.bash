#!/bin/bash

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

if [ "$input" = Stereo ] || [ "$input" = stereo ]; then
    rm -rf $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-stereo $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacements with $input camera is done"
elif [ "$input" = rgbd ] || [ "$input" = RGBD ] ; then
    rm -rf $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-rgbd $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacements with $input camera is done"
else
    echo "Invalid input, please try again."
    exit 1
fi
