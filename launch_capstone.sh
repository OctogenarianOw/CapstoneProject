#!/bin/bash
# Launch script for Capstone UAV simulation

# Kill any existing instances first
echo "Cleaning up existing processes..."
pkill -9 px4 2>/dev/null
pkill -9 gz 2>/dev/null
pkill -9 ruby 2>/dev/null
pkill -9 MicroXRCEAgent 2>/dev/null
pkill -9 ros2 2>/dev/null
sleep 3
echo "Done. Launching simulation..."

wt.exe \
  new-tab --title "PX4+Gazebo" -- wsl.exe -e bash -c "cd ~/PX4-Autopilot && PX4_GZ_WORLD=coastal_world PX4_GZ_MODEL_POSE='0,-5,0.1,0,0,0' make px4_sitl gz_x500_lidar_3d" \; \
  new-tab --title "DDS Agent" -- wsl.exe -e bash -c "sleep 10 && MicroXRCEAgent udp4 -p 8888" \; \
  new-tab --title "LiDAR Bridge" -- wsl.exe -e bash -c "sleep 15 && source ~/ros2_ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'" \; \
  new-tab --title "Terrain Scoring" -- wsl.exe -e bash -c "sleep 18 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation terrain_scoring_node" \; \
  new-tab --title "Waypoint Node" -- wsl.exe -e bash -c "sleep 22 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation waypoint_node" \; \
  new-tab --title "RViz2" -- wsl.exe -e bash -c "sleep 20 && source ~/ros2_ws/install/setup.bash && rviz2 --display-config /home/doria/ros2_ws/src/uav_navigation/config/capstone.rviz"

