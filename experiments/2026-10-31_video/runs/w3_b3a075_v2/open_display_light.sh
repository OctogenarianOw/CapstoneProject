#!/bin/bash
# Open RViz at the lowest CPU priority, its software renderer limited to
# four threads. Measured need (1 Nov, map.log of the 31 Oct takes): on W3
# the filmed missions fused 44 to 96 % of their LiDAR scans with the last
# received pose instead of the pose at the scan instant (812 of 1860 in
# w3_b3a075), against 17 of 161640 in the headless W3 campaign; RViz
# rendering in llvmpipe on all 16 cores starves Gazebo and PX4, and the
# map loses the P5 correction. With nice 19 the simulation gets the CPU
# first and RViz drops frames instead. Same wrapper role as
# open_display_video.sh: no cleanup pattern on the session command line.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
export LP_NUM_THREADS=4
exec nice -n 19 rviz2 -d "$1"
