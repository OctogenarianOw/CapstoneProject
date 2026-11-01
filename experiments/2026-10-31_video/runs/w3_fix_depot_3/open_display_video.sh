#!/bin/bash
# Open RViz on the configuration given as argument. A wrapper so that the
# session command line holds no cleanup pattern of sim_session.sh.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
exec rviz2 -d "$1"
