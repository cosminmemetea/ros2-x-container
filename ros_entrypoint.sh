#!/bin/bash
set -e

# Source ROS și workspace-ul
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Execută comanda primită (e.g., CMD din Dockerfile sau override din docker run)
exec "$@"