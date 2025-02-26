#!/bin/bash

echo "Start Modifications for running Simulation"
input=$1
WORKDIR=/px4
echo "Selected Camera Type: $input"

if [ "$input" = Stereo ] || [ "$input" = stereo ]; then
    rm -rf $WORKDIR/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt \
        $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/CMakeLists.txt $WORKDIR/PX4-Autopilot/src/modules/simulation/gz_bridge/
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-stereo $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacments with $input camera is done"
elif [ "$input" = rgbd ] || [ "$input" = RGBD ] ; then
    rm -rf  $WORKDIR/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt \
            $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    cp -rv $WORKDIR/volume/includes/gz/CMakeLists.txt $WORKDIR/PX4-Autopilot/src/modules/simulation/gz_bridge/
    cp -rv $WORKDIR/volume/includes/gz/models/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/
    cp -rv $WORKDIR/volume/includes/gz/worlds/* $WORKDIR/PX4-Autopilot/Tools/simulation/gz/worlds/
    mv -v  $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-rgbd $WORKDIR/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
    echo "Replacments with $input camera is done"
else
    echo "Invalid input, please try again."
    exit 1
fi



# while true; do
#     echo "For Stereo: 1  FOR RGBD: 2"
#     read input

#     if [ "$input" -eq 1 ]; then
#         rm -rf /PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt \
#               /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
#         cp -rv /volume/includes/gz/CMakeLists.txt /PX4-Autopilot/src/modules/simulation/gz_bridge/
#         cp -rv /volume/includes/gz/models/* /PX4-Autopilot/Tools/simulation/gz/models/
#         cp -rv /volume/includes/gz/worlds/* /PX4-Autopilot/Tools/simulation/gz/worlds/
#         mv -v /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-stereo /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
#         echo "Replacments with stereo camera is done"
#         break
#     elif [ "$input" -eq 2 ]; then
#         rm -rf /PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt \
#               /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
#         cp -rv /volume/includes/gz/CMakeLists.txt /PX4-Autopilot/src/modules/simulation/gz_bridge/
#         cp -rv /volume/includes/gz/models/* /PX4-Autopilot/Tools/simulation/gz/models/
#         cp -rv /volume/includes/gz/worlds/* /PX4-Autopilot/Tools/simulation/gz/worlds/
#         mv -v /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-rgbd /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
#         echo "Replacments with rgbd camera is done"
#         break
#     else
#         echo "Invalid input, please try again."
#     fi
# done