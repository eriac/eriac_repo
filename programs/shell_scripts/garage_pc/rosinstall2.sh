cd 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd src
git clone https://github.com/robosavvy/vive_ros.git
cd ~/catkin_ws
catkin_make



