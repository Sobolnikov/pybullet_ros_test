#!/bin/bash

xhost +local:docker

docker run --device=/dev/dri:/dev/dri \
            -e DISPLAY=unix:0 \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /dev/input:/dev/input \
            --device-cgroup-rule='c 13:* rmw' \
            -v $(pwd):/pybullet_ros_test \
            -it \
            --name pybullet_ros_test \
            --net host \
            ss_ros_noetic