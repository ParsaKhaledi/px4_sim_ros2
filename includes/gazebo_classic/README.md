#  Gazebo Classic
PX4-Autopilot has changed so that iris drone can have stereo cameras and depth camera with ros2 compatible plugins as a sample. Also, apt world was added to this repo to be able to fly inside a house for mapping and exploration purposes. Beside in startFiles directory, you can use gazebo_modification.sh to modify worlds and models.
## Adding your own model 
For adding your own model, you have to go to these 3 locations and add your model:
 - ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/ (Add your model here)
 - ./src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake (Add your model to Cmake list)
 - ./ROMFS/px4fmu_common/init.d-posix/airframes/ (create a new file like samples available and then add it to Cmakelists.txt)

## Adding your own worlds
For adding your own world, you have to go to these locations and add your world:
- ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds (Add your world file here)
- ./src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake (Add your world to Cmake list)