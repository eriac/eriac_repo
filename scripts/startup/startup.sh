#!/bin/bash

# basic
sudo apt install -y git vim openssh-server terminator htop clang-format gtkterm 

# ros basic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt -y update
sudo apt -y upgrade

sudo apt -y install ros-kinetic-desktop-full
apt-cache search ros-kinetic

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash

sudo apt -y install python-rosinstall

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ros additional pkgs

## common
sudo apt install -y ros-kinetic-urdf-tutorial 
sudo apt install -y ros-kinetic-jsk-visualization

## operation
sudo apt install -y ros-kinetic-joy
sudo apt install -y ros-kinetic-joystick-drivers


# sim
sudo apt install -y ros-kinetic-gazebo-ros
sudo apt install -y ros-kinetic-gazebo-ros-control 
sudo apt install -y ros-kinetic-ros-control
sudo apt install -y ros-kinetic-ros-controllers

# camera
sudo apt install -y ros-kinetic-uvc-camera
sudo apt install -y ros-kinetic-image-transport
sudo apt install -y ros-kinetic-image-transport-plugins
sudo apt install -y ros-kinetic-camera-calibration
sudo apt install -y ros-kinetic-image-proc
sudo apt install -y rosros-kinetic-ar-track-alvar

# web
sudo apt install -y ros-kinetic-roswww
sudo apt install -y ros-kinetic-rosbridge-suite 
sudo apt install -y ros-kinetic-web-video-server

# nav
sudo apt install -y ros-kinetic-robot-localization
sudo apt install -y ros-kinetic-gmapping
sudo apt install -y ros-kinetic-amcl
sudo apt install -y ros-kinetic-map-server
sudo apt install -y ros-kinetic-move-base

# other (not ROS)
sudo apt install -y chrony
sudo apt install -y libarmadillo-dev libarmadillo6
sudo apt install -y lpc21isp
