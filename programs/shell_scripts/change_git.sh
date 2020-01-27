#!/bin/bash

if [ $1 = "eriac" ] ; then
  rm -rf .ssh
  cp -r .ssh_eriac .ssh
  echo "cp -r .ssh_eriac .ssh"
  ssh-add ~/.ssh/id_rsa
elif [ $1 = "srs" ] ; then
  rm -rf .ssh
  cp -r .ssh_srs .ssh
  echo "cp -r .ssh_srs .ssh"
  ssh-add ~/.ssh/id_rsa
else
  echo "error"
fi
