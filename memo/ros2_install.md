https://docs.ros.org/en/humble/Installation.html

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash

# gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
export SVGA_VGPU10=0

# next 
ssh-keygen
cat ~/.ssh/id_rsa.pub

sudo apt install vim git
sudo apt install python3-colcon-common-extensions
# sudo apt install ros-humble-gazebo-plugins
# sudo apt install ros-humble-laser-filters
# sudo apt install ros-humble-xacro
# sudo apt install ros-humble-slam-toolbox

# for gazebo sim
source /usr/share/gazebo/setup.bash

# rosdep
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update

# mavros
$ cd /opt/ros/humble/lib/mavros
$ sudo ./install_geographiclib_datasets.sh 
sudo usermod -aG dialout $USER

# ydlidar
cd 
git clone https://github.com/YDLIDAR/YDLidar-SDK
mkdir -p YDLidar-SDK/build
cd YDLidar-SDK/build
cmake ..
make
sudo make install

