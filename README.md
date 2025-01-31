# px4_sim_ros2

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.
This repo Contains a ci platform for simulating px4 drones with gazebo with docker image and container with nvidia graphic card capabilites. There are some startFiles which can initiate different tools and Parameters for running simulation with different tools. Base on discribtion below you can add camera for image prossecing purposes.
# Infrastructure
The Dockerfiles are used to build docker images in infrastructure branch and then copied here. You can check [my docker hub](https://hub.docker.com/r/alienkh/px4_sim) for download docker image. 
## ROS2 Cyclonedds
Many repos like rtabmap recommends to replace default dds of ROS2 with Cyclonedds. Its easy to use other ddses. All you need is to install them and do an export in your environment.
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
# Add models and worlds in gazebo classic
PX4-Autopilot has changed so that iris drone can have stereo cameras and depth camera with ros2 compatible plugins. Also, apt world was added to this repo to be able to fly inside a house for mapping and exploration purposes. Beside in startFiles directory, you can use gazebo_modification.sh to modify worlds and models.
## Adding your own model 
For adding your own model, you have to go to these 3 locations and add your model:
 - ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/ (Add your model here)
 - ./src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake (Add your model to Cmake list)
 - ./ROMFS/px4fmu_common/init.d-posix/airframes/ (create a new file like samples available and then add it to Cmakelists.txt)

## Adding your own worlds
For adding your own world, you have to go to these locations and add your world:
- ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds (Add your world file here)
- ./src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake (Add your world to Cmake list)

# Add models and worlds in gz 
Discribtion will be added here soon ...
In include folder there is a file named replacments.bash which you can check it and volume mounted in docker-compose file to see where to copy include folder and run the very script to add word to px4.

