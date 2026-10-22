#!/bin/bash
# Open the demonstration panel (demo_panel.py) with ROS and the workspace
# sourced. Its command line holds none of the cleanup patterns of
# sim_session.sh, so ending a session leaves the panel open for the next.
#   bash ~/ros2_ws/droneAfter/experiments/demo_panel.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
exec python3 "$(cd "$(dirname "$0")" && pwd)/demo_panel.py"
