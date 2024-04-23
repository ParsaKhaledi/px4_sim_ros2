#!/bin/bash
sudo apt install -y xorg openbox xauth
# For Use Nvidia in docker file install nvidia-container-toolkit and nvidia-docker2 plus nvidia driver
#   https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
git clone https://github.com/ParsaKhaledi/px4_sim_ros2.git 
docker run -it --name px4_sim --network host --privileged --gpus all --runtime=nvidia \
    --env="XAUTHORITY=$XAUTH" -v /dev/:/dev/ -v /tmp/.X11-unix/:/tmp/.X11-unix/ -e DISPLAY=$DISPLAY \
    -v ./px4_sim_ros2:/px4_sim_ros2/ alienkh/px4_sim:latest bash
