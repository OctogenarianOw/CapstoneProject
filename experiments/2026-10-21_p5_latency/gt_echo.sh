#!/bin/bash
# True poses of the world with their simulation stamps, to truth.jsonl in
# the run directory: the ROS bridge drops the names and the stamps of
# Pose_V (g1), Gazebo's own echo keeps them. A script of its own so that
# no gz command sits on the command line of sim_session.sh.
exec gz topic -e -t /world/pits/dynamic_pose/info --json-output > truth.jsonl
