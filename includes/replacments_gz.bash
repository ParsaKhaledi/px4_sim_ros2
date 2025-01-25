#!/bin/bash

rm -f /PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt
cp -r /volume/includes/gz/CMakeLists.txt /PX4-Autopilot/src/modules/simulation/gz_bridge/
cp -r /volume/includes/gz/models/* /PX4-Autopilot/Tools/simulation/gz/models/
cp -r /volume/includes/gz/worlds/* /PX4-Autopilot/Tools/simulation/gz/worlds/
echo "Done."