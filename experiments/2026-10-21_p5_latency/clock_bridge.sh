#!/bin/bash
# Gazebo clock to ROS, for the bag of the P5 latency flights: the lidar
# stamps are simulation time, the PX4 stamps system time (measured on
# r1: a pose arrives 5 ms after its stamp), and /clock gives the offset
# between the two clocks, a small message whose transport is negligible.
# A script of its own because its command holds "parameter_bridge", a
# cleanup pattern of sim_session.sh that would kill the session if
# passed on its command line.
exec ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
