sudo apt install python-bloom

bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro melodic
fakeroot debian/rules binary

sudo apt install ./ros-melodic-s4-msgs_0.0.0-0bionic_amd64.deb 
sudo apt purge ros-melodic-s4-msgs
