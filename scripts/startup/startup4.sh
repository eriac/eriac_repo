#!/bin/bash

sudo cat << EOF | sudo tee -a /etc/hosts > /dev/null
192.168.2.61    remote1
192.168.2.62    remote2
192.168.2.63    remote3
192.168.2.71    OptiPlex-9010
EOF
