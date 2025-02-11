#!/bin/bash
while true; do
    echo "For Stereo: 1  FOR RGBD: 2"
    read input

    if [ "$input" -eq 1 ]; then
        rm -f /PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt
        cp -r /volume/includes/gz/CMakeLists.txt /PX4-Autopilot/src/modules/simulation/gz_bridge/
        cp -r /volume/includes/gz/models/* /PX4-Autopilot/Tools/simulation/gz/models/
        cp -r /volume/includes/gz/worlds/* /PX4-Autopilot/Tools/simulation/gz/worlds/
        mv    /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-stereo /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
        echo "Replacments with stereo camera is done"
        break
    elif [ "$input" -eq 2 ]; then
        rm -f /PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt
        cp -r /volume/includes/gz/CMakeLists.txt /PX4-Autopilot/src/modules/simulation/gz_bridge/
        cp -r /volume/includes/gz/models/* /PX4-Autopilot/Tools/simulation/gz/models/
        cp -r /volume/includes/gz/worlds/* /PX4-Autopilot/Tools/simulation/gz/worlds/
        mv    /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite-rgbd /PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite
        echo "Replacments with rgbd camera is done"
        break
    else
        echo "Invalid input, please try again."
    fi
done