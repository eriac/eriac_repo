#!/bin/bash

echo "export ROS_MASTER_URI=http://`hostname -I | cut -d' ' -f1`:11311" >> ~/.bashrc
echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> ~/.bashrc

