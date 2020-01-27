#prepare for vive_ros
sudo apt-get -y install git cmake libsdl2-dev libglew-dev
cd ~
mkdir libraries
cd libraries
git clone https://github.com/ChristophHaag/openvr.git
cd openvr
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make

#vive useb should be 
sudo echo 'KERNEL=="hidraw*", ATTRS{idVendor}=="0bb4", MODE="0666"' | sudo tee /etc/udev/rules.d/88-vive.rules
sudo echo 'KERNEL=="hidraw*", ATTRS{idVendor}=="28de", MODE="0666"' | sudo tee -a /etc/udev/rules.d/88-vive.rules
sudo echo 'KERNEL=="hidraw*", ATTRS{idVendor}=="0424", MODE="0666"' | sudo tee -a /etc/udev/rules.d/88-vive.rules
sudo /etc/init.d/udev restart

#install steam, after this hop up GUI
sudo dpkg --add-architecture i386
sudo apt-get -y update
sudo apt-get -y install wget gdebi libgl1-mesa-dri:i386 libgl1-mesa-glx:i386 libc6:i386
wget http://media.steampowered.com/client/installer/steam.deb
sudo gdebi steam.deb
steam
