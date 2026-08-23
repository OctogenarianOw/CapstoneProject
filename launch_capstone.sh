#!/bin/bash
echo "Cleaning up existing processes..."
pkill -9 px4 2>/dev/null
pkill -9 gz 2>/dev/null
pkill -9 ruby 2>/dev/null
pkill -9 MicroXRCEAgent 2>/dev/null
pkill -9 ros2 2>/dev/null
sleep 3

echo ""
echo "=== Capstone UAV Simulation ==="
echo "1 - Gazebo + RViz2   full visuals"
echo "2 - Headless + RViz2 no Gazebo window, lower CPU"
echo "3 - Headless only    no visuals at all"
echo ""
read -p "Choose 1, 2 or 3: " choice

if [ "$choice" = "1" ]; then
    PX4_CMD="cd ~/PX4-Autopilot && PX4_GZ_WORLD=coastal_world PX4_GZ_MODEL_POSE=0,-5,0.1,0,0,0 make px4_sitl gz_x500_lidar_3d"
elif [ "$choice" = "2" ] || [ "$choice" = "3" ]; then
    PX4_CMD="cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=coastal_world PX4_GZ_MODEL_POSE=0,-5,0.1,0,0,0 make px4_sitl gz_x500_lidar_3d"
else
    echo "Invalid choice, launching in full mode by default"
    PX4_CMD="cd ~/PX4-Autopilot && PX4_GZ_WORLD=coastal_world PX4_GZ_MODEL_POSE=0,-5,0.1,0,0,0 make px4_sitl gz_x500_lidar_3d"
fi

echo "Done. Launching simulation..."

if [ "$choice" = "3" ]; then
    wt.exe new-tab --title "PX4+Gazebo" -- wsl.exe -e bash -c "$PX4_CMD" \; new-tab --title "DDS Agent" -- wsl.exe -e bash -c "sleep 6 && MicroXRCEAgent udp4 -p 8888" \; new-tab --title "LiDAR Bridge" -- wsl.exe -e bash -c "sleep 10 && source ~/ros2_ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \; new-tab --title "Terrain Scoring" -- wsl.exe -e bash -c "sleep 12 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation terrain_scoring_node" \; new-tab --title "LiDAR Processor" -- wsl.exe -e bash -c "sleep 12 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation lidar_processor_node" \; new-tab --title "Obstacle Layer" -- wsl.exe -e bash -c "sleep 13 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation obstacle_layer_node" \; new-tab --title "Terrain Follower" -- wsl.exe -e bash -c "sleep 13 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation terrain_follower_node" \; new-tab --title "Frontier Detector" -- wsl.exe -e bash -c "sleep 14 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation frontier_detector_node" \; new-tab --title "A* Planner" -- wsl.exe -e bash -c "sleep 14 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation astar_planner_node" \; new-tab --title "Watchdog" -- wsl.exe -e bash -c "sleep 14 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation path_watchdog_node" \; new-tab --title "Waypoint Node" -- wsl.exe -e bash -c "sleep 15 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation waypoint_node" \; new-tab --title "Manager" -- wsl.exe -e bash -c "sleep 17 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation exploration_manager_node"
else
    wt.exe new-tab --title "PX4+Gazebo" -- wsl.exe -e bash -c "$PX4_CMD" \; new-tab --title "DDS Agent" -- wsl.exe -e bash -c "sleep 6 && MicroXRCEAgent udp4 -p 8888" \; new-tab --title "LiDAR Bridge" -- wsl.exe -e bash -c "sleep 10 && source ~/ros2_ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \; new-tab --title "Terrain Scoring" -- wsl.exe -e bash -c "sleep 12 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation terrain_scoring_node" \; new-tab --title "LiDAR Processor" -- wsl.exe -e bash -c "sleep 12 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation lidar_processor_node" \; new-tab --title "Obstacle Layer" -- wsl.exe -e bash -c "sleep 13 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation obstacle_layer_node" \; new-tab --title "Terrain Follower" -- wsl.exe -e bash -c "sleep 13 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation terrain_follower_node" \; new-tab --title "Frontier Detector" -- wsl.exe -e bash -c "sleep 14 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation frontier_detector_node" \; new-tab --title "A* Planner" -- wsl.exe -e bash -c "sleep 14 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation astar_planner_node" \; new-tab --title "Watchdog" -- wsl.exe -e bash -c "sleep 14 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation path_watchdog_node" \; new-tab --title "Waypoint Node" -- wsl.exe -e bash -c "sleep 15 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation waypoint_node" \; new-tab --title "Manager" -- wsl.exe -e bash -c "sleep 17 && source ~/ros2_ws/install/setup.bash && ros2 run uav_navigation exploration_manager_node" \; new-tab --title "RViz2" -- wsl.exe -e bash -c "sleep 13 && source ~/ros2_ws/install/setup.bash && rviz2 --display-config /home/doria/ros2_ws/src/uav_navigation/config/capstone.rviz"
fi
