#!/bin/bash
sudo apt install -y xorg openbox xauth
# For Use Nvidia in docker file install nvidia-container-toolkit and nvidia-docker2 plus nvidia driver
#   https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

docker run -it --name px4_python --network host --privileged --gpus all --runtime=nvidia \
    --env="XAUTHORITY=$XAUTH" -v /dev/:/dev/ -v /tmp/.X11-unix/:/tmp/.X11-unix/ -e DISPLAY=$DISPLAY \
    -v ~/docker_ws/px4_sim_volume:/px4_sim_volume/ px4_simulation_1.14:latest bash
