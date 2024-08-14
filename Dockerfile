FROM ros:humble-perception-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV ROS2_INSTALL_PATH=/opt/ros/$ROS_DISTRO
WORKDIR /px4_sim_ros2

RUN apt-get clean
RUN apt update && apt -y upgrade && \
	apt install -y --no-install-recommends \
	vim udev git sudo unzip curl cmake wget tmux \
	ros-dev-tools sudo \
	gnupg2 lsb-release \
	xauth xorg openbox python3-argcomplete python3 python3-pip && \
	wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \ 
	sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
	apt update && apt install -y --no-install-recommends \
	gz-harmonic ros-${ROS_DISTRO}-ros-gz

RUN rosdep update

# RUN pip3 install --user -U pyros-genmsg jsonschema jinja2 colcon-ros kconfiglib scipy

## PX4 Stuff
RUN cd / && git clone --recursive --progress --verbose https://github.com/PX4/PX4-Autopilot
RUN cd /PX4-Autopilot &&\
	# git checkout cea185268  &&\
	# git submodule update --init --recursive &&\
	./Tools/setup/ubuntu.sh
# RUN apt install -y --no-install-recommends gcc-arm-none-eabi

## Install Nav2 and rtabmap
# RUN apt install -y  \
# 	ros-$ROS_DISTRO-navigation2 \
# 	ros-$ROS_DISTRO-nav2-bringup \
# 	ros-$ROS_DISTRO-rtabmap-ros 
	# ros-$ROS_DISTRO-gazebo*

## Install Micro XRCE-DDS
RUN cd / && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
RUN cd /Micro-XRCE-DDS-Agent &&\
	mkdir build && cd build && cmake .. &&\
	make && make install && sudo ldconfig /usr/local/lib/ &&\
	cd / 
	
# RUN pip3 install -U pyros-genmsg setuptools


### Build WS (px4_ros_com && px4_msgs && m-explore-ros2)
RUN mkdir -p /ws_px4/src
RUN cd /ws_px4/src && git clone --progress --verbose https://github.com/PX4/px4_msgs.git
RUN cd /ws_px4/src && git clone --progress --verbose https://github.com/PX4/px4_ros_com.git
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /ws_px4 && colcon build

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
RUN echo "source /ws_px4/install/setup.bash" >> ~/.bashrc

### Finilize
RUN rm -rf /var/lib/apt/lists/* 