#!/bin/bash
# Open the RViz display of the exploration with its saved configuration.
# A wrapper so that the session command line holds no cleanup pattern.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
exec rviz2 -d ~/ros2_ws/droneAfter/install/uav_core/share/uav_core/rviz/exploration.rviz
