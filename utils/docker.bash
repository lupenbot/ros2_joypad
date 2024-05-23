#/bin/bash -c

# Script for running a docker container to test
# ROS2 pkg.

# Variables.
DOCKER_ROS_NETWORK=host
DOCKER_ROS_IMAGE_TAG=ros:humble-ros-base
DOCKER_ROS_WORKSPACE=/ros_ws

# Run container
docker run -it --rm --privileged \
-v /dev/input:/dev/input \
-v /dev/shm:/dev/shm \
-v .:$DOCKER_ROS_WORKSPACE/src/ros2_pkg \
-w $DOCKER_ROS_WORKSPACE \
--network $DOCKER_ROS_NETWORK \
--name ros2-container $DOCKER_ROS_IMAGE_TAG bash