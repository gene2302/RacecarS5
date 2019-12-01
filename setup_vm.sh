#!/usr/bin/env bash

# Phase 1 - ROS packages

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install -y ros-melodic-desktop-full \
                    ros-melodic-gazebo-ros \
                    ros-melodic-ros-controllers \
                    ros-melodic-gazebo-ros-control \
                    ros-melodic-ackermann-msgs \
                    ros-melodic-joy \
                    ros-melodic-rosserial-python \
                    ros-melodic-imu-filter-madgwick \
                    ros-melodic-robot-localization \
                    ros-melodic-move-base \
                    ros-melodic-global-planner \
                    ros-melodic-teb-local-planner \
                    ros-melodic-ackermann-msgs \
                    ros-melodic-rtabmap-ros \
                    ros-melodic-robot-localization \
                    ros-melodic-rosbridge-server  \
                    ros-melodic-web-video-server \
                    ros-melodic-roswww \
                    vim \
                    git \
                    net-tools \
                    nodejs \
                    sed 

# Phase 2 - Workspace setup
source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# Phase 3- ROS Build
cd ~/catkin_ws
catkin_make

# Phase 4 - For automatic sourcing of ROS scripts when starting a new session
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# VMWare GPU acceleration disabling (optional)
echo "export SVGA_VGPU10=0" >> ~/.bashrc
