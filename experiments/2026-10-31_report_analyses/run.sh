#!/bin/bash
# Run one script of this directory with ROS and the workspace sourced
# (the map panels import uav_core). Usage: bash run.sh fig_worlds.py
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
cd "$(dirname "$0")"
python3 "$@"
