#FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu20.04
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu22.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV LANG=en_US.UTF-8
# ENV ROS_DISTRO=foxy
ENV ROS_DISTRO=humble
ENV ROS2_INSTALL_PATH=/opt/ros/$ROS_DISTRO
ENV WS_PX4=/ws_px4
WORKDIR /px4_sim_ros2

RUN apt-get clean
RUN apt update && apt -y upgrade
RUN apt install -y --no-install-recommends \
	vim udev git sudo unzip curl cmake wget tmux \
	software-properties-common cmake libgoogle-glog-dev \
	libatlas-base-dev libsuitesparse-dev python3-future \
	ca-certificates devilspie gnupg2 mesa-utils lsb-release \
	xauth xorg openbox python3-argcomplete python3 python3-pip
RUN	add-apt-repository -y universe && apt update

# # <Manually Get Source (we had filtering problems therefore we download it manually and Copy it :) )>
# COPY ./NICE-GPG-KEY /
# COPY ./nice-dcv-2023.0-15487-ubuntu2204-x86_64.tgz /
# RUN cd /
# RUN gpg --import NICE-GPG-KEY
# RUN tar xzf nice-dcv-2023.0-15487-ubuntu2204-x86_64.tgz && \
#         cd nice-dcv-2023.0-15487-ubuntu2204-x86_64 && \
#         apt install ./nice-dcv-web-viewer_2023.0.15487-1_amd64.ubuntu2204.deb
# RUN rm -rf nice-dcv-2023.0-15487-ubuntu2204-x86_64.tgz nice-dcv-2023.0-15487-ubuntu2204-x86_64


# < Directly Get source >
RUN wget https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY && gpg --import NICE-GPG-KEY && \
        wget https://d1uj6qtbmh3dt5.cloudfront.net/2023.1/Servers/nice-dcv-2023.1-16388-ubuntu2204-x86_64.tgz && \
        tar -xvzf nice-dcv-2023.1-16388-ubuntu2204-x86_64.tgz && \
		cd nice-dcv-2023.1-16388-ubuntu2204-x86_64 && \
		apt install -y ./nice-dcv-server_2023.1.16388-1_amd64.ubuntu2204.deb

## INSTALL ROS2
RUN apt update && apt -y install locales && \
        locale-gen en_US en_US.UTF-8 && \
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
        apt update &&\
        apt install -y ros-$ROS_DISTRO-desktop ros-dev-tools python3-argcomplete

### Install Gazebo and some Reqs
# RUN curl -sSL http://get.gazebosim.org | sh
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt update && apt install -y gazebo libgazebo-dev

RUN rosdep init && rosdep update
RUN apt install -y --no-install-recommends \
	ros-$ROS_DISTRO-gazebo-dev ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-gazebo-plugins-dbgsym \
	ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-dbgsym ros-$ROS_DISTRO-gazebo-ros-pkgs \
	ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros2-control-dbgsym \
	ros-$ROS_DISTRO-gazebo-ros2-control-demos ros-$ROS_DISTRO-gazebo-ros2-control-demos-dbgsym
RUN pip3 install --user -U pyros-genmsg jsonschema jinja2 colcon-ros kconfiglib scipy

## PX4 Stuff
RUN cd / && git clone --recursive --progress --verbose https://github.com/PX4/PX4-Autopilot -b v1.14.4
RUN cd /PX4-Autopilot &&\
	# git checkout cea185268  &&\
	# git submodule update --init --recursive &&\
	./Tools/setup/ubuntu.sh
# RUN apt install -y --no-install-recommends gcc-arm-none-eabi

## Install Nav2 and rtabmap
RUN apt install -y  \
	ros-$ROS_DISTRO-navigation2 \
	ros-$ROS_DISTRO-nav2-bringup \
	ros-$ROS_DISTRO-rtabmap-ros 
	# ros-$ROS_DISTRO-gazebo*

## Install Micro XRCE-DDS
RUN cd / && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
RUN cd /Micro-XRCE-DDS-Agent &&\
	mkdir build && cd build && cmake .. &&\
	make -j $12 && make install && sudo ldconfig /usr/local/lib/ &&\
	cd / 

# ### Install micro_rtps_agent for PX4 V1.13 only

# ## Install Foonathan_memory
# RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git &&\
# 	cd foonathan_memory_vendor &&\
# 	mkdir build && cd build  &&\
# 	cmake ..  &&\
# 	cmake --build . --target install

# ## Install Gradle
# RUN curl -s "https://get.sdkman.io" | bash &&\
# 	source "$HOME/.sdkman/bin/sdkman-init.sh" &&\
# 	sdk install gradle 6.3

# ## Install FAST-RTPS
# RUN git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b 1.8.x &&\
# 	cd Fast-RTPS && mkdir build && cd build && cmake -DTHIRDPARTY=ON -DSECURITY=ON .. &&\
# 	make && make install
# ### Install FAST-RTPS-Gen
# RUN git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.4 /Fast-RTPS-Gen 
# RUN source "$HOME/.sdkman/bin/sdkman-init.sh" && cd /Fast-RTPS-Gen && gradle assemble && gradle install

# ## Install FAST-DDS
# RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 ~/FastDDS-2.0.2 && \
# 	cd ~/FastDDS-2.0.2 && \
# 	mkdir build && cd build && \
# 	cmake -DTHIRDPARTY=ON -DSECURITY=ON .. && \
# 	make -j$(nproc --all) && \
# 	make install
	
RUN pip3 install -U pyros-genmsg setuptools


### Build WS (px4_ros_com && px4_msgs && m-explore-ros2)
RUN mkdir -p $WS_PX4/src &&\
	cd $WS_PX4/src && git clone --progress --verbose https://github.com/PX4/px4_msgs.git -b release/1.14 
	# source /opt/ros/$ROS_DISTRO/setup.bash && cd $WS_PX4 && colcon build 

# RUN cd /px4_sim_ros2/src && git clone --progress --verbose https://github.com/robo-friends/m-explore-ros2.git
# RUN cd / && git clone --progress --verbose https://github.com/ParsaKhaledi/px4_sim_ros2.git

# RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /ws_px4_ros2/src/px4_ros_com/scripts/ && source build_ros2_workspace.bash

### Tumux conf
RUN cd ~ && git clone https://github.com/gpakosz/.tmux.git &&\
	ln -s -f .tmux/.tmux.conf &&\
	cp .tmux/.tmux.conf.local . &&\
	echo "set -g mouse on" >> ~/.tmux.conf 

### Write in ~/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $WS_PX4/install/setup.bash" >> ~/.bashrc

### Finilize
RUN rm -rf /var/lib/apt/lists/* 
