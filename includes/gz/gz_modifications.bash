#!/bin/bash
# Overlay camera models, or leave the stock PX4 model for a flight-only start.

echo "Start Modifications for running Simulation"
input=$1
WORKDIR=/home/px4
# Parameter files are applied by gz_start_px4_gz_sim.sh after the firmware
# build, so a rebuilt rootfs cannot drop the airframe .post scripts.
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

# Software-rendered CI flights pass GZ_CAMERA_UPDATE_RATE. Rewrite the copy
# under the PX4 tree only. The Oak-D files in this repo stay unchanged.
if [ -n "${GZ_CAMERA_UPDATE_RATE:-}" ] && [ -f "$WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf" ]; then
    echo "Software camera overlay: ${GZ_CAMERA_UPDATE_RATE} Hz. Image size stays with the model."
    python3 "$WORKDIR/volume/includes/gz/tune_gz_cameras.py" \
        --model "$WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf" \
        --rate "${GZ_CAMERA_UPDATE_RATE}"
fi
