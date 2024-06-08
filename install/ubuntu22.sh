#!/usr/bin/sh -e　-x

echo "start install"

# basic 
sudo apt update -y
sudo apt dist-upgrade -y
sudo apt install -y git vim 

# ROS2-desktop
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8 
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y
sudo apt upgrade -y

sudo apt install -y ros-humble-desktop

# ROS2-gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs

# ROS2-catkin
sudo apt install -y python3-colcon-common-extensions

# ROS2-rosdep
sudo apt install -y python3-rosdep2
sudo rosdep init
rosdep update

# special handling mavros
sudo apt install -y ros-humble-mavros
cd /opt/ros/humble/lib/mavros
sudo ./install_geographiclib_datasets.sh

# special handling ydlidar-SDK
cd /tmp
git clone https://github.com/YDLIDAR/YDLidar-SDK
mkdir -p YDLidar-SDK/build
cd YDLidar-SDK/build
cmake .. 
make 
sudo make install

# device related
sudo usermod -aG dialout $USER
