# PX4 Simulation with ROS2
This repository Contains an ecosystem for easily deploy PX4 simulation and developments.
It uses ci platform for buliding and push environment required to simulate px4 drones with gz simulator using docker image and containers (possible to use nvidia graphic cards).  

# Infrastructure
The essential tools required for the hole ecosystem are dicribed in this section.

## Docker
[Docker](https://docs.docker.com/get-started/docker-overview/) is a set of platform as a service products that use OS-level virtualization to deliver software in packages called containers. Docker can be installed from [this link](https://docs.docker.com/engine/). In this project [docker compose](https://docs.docker.com/compose/) is used to manage containers. The Dockerfiles are used to build docker images in infrastructure branch and then copied here. You can check [my docker hub](https://hub.docker.com/r/alienkh/px4_sim) for download docker image sepratly. 

## X11
The X Window System (aka X11) is a client/server network protocol that's been used for decades on a variety of different hardware platforms. It has been implemented by a number of different vendors for a wide variety of hardware platforms. The xserver includes a framework for managing video and input device X drivers. These drivers interface to lower level kernel device drivers (or to the hardware directly in a few cases). Typically these drivers are developed and supported by the hardware vendor in conjunction with the kernel and X.org communitie (Waylands is a new tool for the very use case). Install X11 as fallow:
```
sudo apt-get install xauth xorg openbox
```
## Tmux
[tmux](https://github.com/tmux/tmux/wiki) is a terminal multiplexer. It lets you switch easily between several programs in one terminal, detach them (they keep running in the background) and reattach them to a different terminal. This tool is used to make stop and start each module in containers more easily and make contiloing each part of projects more possible. 

## PX4
[PX4](https://docs.px4.io/v1.15/en/) is the Professional Autopilot. Developed by world-class developers from industry and academia, and supported by an active world wide community, it powers all kinds of vehicles from racing and cargo drones through to ground vehicles and submersibles.The V1.15.3 version of autopilot is used for this project and can be found in [PX4-Autopilot Gtihub](https://github.com/PX4/PX4-Autopilot/tree/v1.15.4).

## QgroundControl
QGroundControl (QGC) is a cross-platform ground control station (GCS) software used for controlling and managing drones, particularly those using PX4 or ArduPilot autopilots. It provides a user-friendly interface for flight control, mission planning, and vehicle setup. To connect to sitl, in QGC’s Application Settings → Comm Links → Add, choose “UDP”—enter the PX4 container’s IP (or hostname) and add these ports to a udp connection:
```
px4_sim:14550
px4_sim:14540
px4_sim:14580
px4_sim:18570
```
These ports can be found in outputs of px4_sitl make command that can be extracted by:
```
docker logs px4_sim 2>&1 | grep mavlink
```

## ROS2
The Robot Operating System [(ROS)](https://www.ros.org/) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source. [Harmonic](https://docs.ros.org/en/humble/index.html) version of ros2 is used for this project.

##  GZ (Recommended)
[GZ harmonic](https://gazebosim.org/docs/harmonic/getstarted/) is used for simulation along PX4 v1.15.4. It is recommended to use this simulation since it is planed to be used in future developments of PX4 and Gazebo.

### Add models and worlds in gz 
Discribtion will be added here soon ...
In include folder there is a file named replacments.bash which you can check it and volume mounted in docker-compose file to see where to copy include folder and run the very script to add word and modify model of PX4-Autopilote.
Files related to gz simulation has been relocated to include folder. Some modifications will happen by running px4_sim_ros2/includes/gz/gz_modifications.bash to add some worlds and camera to x500 model that can be checked in very file. 

## ROS_GZ
This package provides a network bridge which enables the exchange of messages between ROS and Gazebo Transport. in [ros_gz](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge) messeage types can be found plus some examples to check connection between GZ and ROS2. How to write yaml for bridge is also writen. [Husarion](https://github.com/husarion/husarion_gz_worlds) is a good repo for more gz worlds. For Migration from Gazebo-classic [this link](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/) can be beneficial. Documentation of PX4 for gz simulation can be found [here](https://docs.px4.io/main/en/sim_gazebo_gz/#specify-world).

## Running simulation
Duo to some problems including frequency reduction of topics (when using bridge mode network of docker) and ros2 issues in getting topics data from other containers (in host network mode of docker) some services are running in same contianer for now. (The issue may related to my PC performance and bridge is working fine in my other projects).
For running simulation : 
Tmux and Docker needed to be installed. 
```
# Allow docker to use X11 display in host
xhost +Local:*

# To select Camera Type and world : CameraType=rgbd or Stereo
# To select World                 : World=apt_world (Not Working for now -> go to perivious commits)
CameraType=rgbd World=apt docker compose -f docker-compose-px4.yml up -d

# Run Simulation in your host Tmux
CameraType=rgbd World=apt ./includes/gz/startFiles/tmux-session.sh
```
## ROS_bridge for using gz and ros2 topics
For using camera and other data from GZ a gz_bridge has to initiate and all params related to it are written as yaml file and can be run with fallowing command (already exist in docker compose file):
```
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/volume/includes/gz/config_gz_bridge.yaml
```
## ROS2 Cyclonedds
Many repos like rtabmap recommends to replace default dds of ROS2 with Cyclonedds. Its easy to use other ddses. All you need is to install them and do an export in your environment.
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```