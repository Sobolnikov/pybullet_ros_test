#!/bin/bash
apt update
apt -y upgrade

apt install -y nano \
               gdb \
               gdbserver \
               python3-pip \
               jstest-gtk \
               wget \
               git

source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
mkdir -p /root/.config/jstest-gtk