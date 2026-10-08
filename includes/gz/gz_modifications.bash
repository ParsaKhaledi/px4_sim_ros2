#!/bin/bash
# Overlay camera models, or leave the stock PX4 model for a flight-only start.

echo "Start Modifications for running Simulation"
input=$1
WORKDIR=/home/px4
# PX4 1.17 rcS imports parameters.bson, applies PX4_PARAM_* with `param set`,
# then the airframe's `param set-default`. It never sources px4-rc.params,
# and set-default does not override a value that is already set. Headless
# runs pass PX4_PARAM_NAV_DLL_ACT=0 (and NAV_RCL_ACT, COM_RC_IN_MODE,
# COM_RC_LOSS_T) on the PX4 service instead.
# Camera Modifications:
echo "Selected Camera Type: $input"

if [ "$input" = none ] || [ "$input" = flight ]; then
    echo "Flight-only start. Stock PX4 model, no camera overlay."
elif [ "$input" = Stereo ] || [ "$input" = stereo ]; then
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
