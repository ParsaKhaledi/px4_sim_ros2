#!/bin/bash
# update and upgrade OS
apt-get update
apt-get upgrade -y
apt install -y vim sudo bash-completion git locales \
    curl software-properties-common 
# add-apt-repository --yes --update ppa:ansible/ansible
# apt install -y ansible
## INSTALL ROS2 Humble
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && \
    echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locales
sudo apt install ros-humble-desktop ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
## Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh &&
sh get-docker.sh <<< y
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker


## Create ssh-keygen
# ssh-keygen -q -t rsa -N '' -f ~/.ssh/id_rsa <<<y >/dev/null 2>&1
# For overwrite :
# ssh-keygen -q -t rsa -N '' <<< $'\ny' >/dev/null 2>&1
