#!/bin/bash

rm -f /PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt
cp -r /volume/includes/CMakeLists.txt /PX4-Autopilot/src/modules/simulation/gz_bridge/
cp -r /volume/husarion_gz_worlds/models/* /PX4-Autopilot/Tools/simulation/gz/models/
cp -r /volume/husarion_gz_worlds/worlds/* /PX4-Autopilot/Tools/simulation/gz/worlds/
echo "Done."