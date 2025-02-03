#!/bin/bash

xhost +local:docker
docker container start pybullet_ros_test
docker exec -it pybullet_ros_test bash