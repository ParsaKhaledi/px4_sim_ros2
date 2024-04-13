# px4_sim_ros2

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

# Add models and worlds
PX4-Autopilot has changed so that iris drone can have stereo cameras and depth camera with ros2 compatible plugins. Also, apt world was added to this repo to be able to fly inside a house for mapping and exploration purposes.
## Adding your own model
For adding your own model, you have to go to these 3 locations and add your model:
 - ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/ (Add your model here)
 - ./src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake (Add your model to Cmake list)
 - ./ROMFS/px4fmu_common/init.d-posix/airframes/ (create a new file like samples available and then add it to Cmakelists.txt)

## Adding your own worlds
For adding your own world, you have to go to these locations and add your world:
- ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds (Add your world file here)
- ./src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake (Add your world to Cmake list)


<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>
<div style="padding:10px">&nbsp;</div>
