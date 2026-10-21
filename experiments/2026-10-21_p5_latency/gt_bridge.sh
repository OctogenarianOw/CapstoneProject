#!/bin/bash
# Gazebo clock and true pose of the aircraft to ROS, for the ground-truth
# flight of the P5 latency measurement (run_gt.sh): the odometry the x500
# model publishes, and the dynamic poses of the world as a fallback if
# the first mapping is refused. A script of its own because its command
# holds "parameter_bridge", a cleanup pattern of sim_session.sh.
exec ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/model/x500_slz_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance' \
  '/world/pits/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
