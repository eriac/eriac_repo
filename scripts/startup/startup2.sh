#!/bin/bash

sudo gpasswd --add $USER dialout

sudo cat << EOF | sudo tee /etc/udev/rules.d/80-srs.rules > /dev/null
# udev rule for FT231X (Dual USB-UART/FIFO IC)
ACTION=="add", SUBSYSTEMS=="usb", ATTRS{manufacturer}=="SRS", ATTRS{product}=="ydlidar", SYMLINK+="ydlidar"
ACTION=="add", SUBSYSTEMS=="usb", ATTRS{manufacturer}=="SRS", ATTRS{product}=="CANadapter", SYMLINK+="CANadapter"
ACTION=="add", SUBSYSTEMS=="usb", ATTRS{manufacturer}=="SRS", ATTRS{product}=="IMUadapter", SYMLINK+="IMUadapter"
EOF
