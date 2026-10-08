#!/bin/bash

echo "Start Modifications for running Simulation"
input=$1
WORKDIR=/home/px4
# PX4 parameters (offboard failsafe + EKF2). Re-running this script replaces
# the marked block instead of appending another copy.
# ESTIMATION_MODE=vision|gps (default vision). See docs/px4_control.md.
# shellcheck disable=SC1091
. "$WORKDIR/volume/includes/gz/params/install_px4_control_params.bash"
install_px4_control_params \
  "$WORKDIR/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params" \
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
