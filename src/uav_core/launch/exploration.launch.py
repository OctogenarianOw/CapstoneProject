"""
ROS side of an exploration session.

Bridges, mapping, energy monitor, executor, manager and display. PX4
SITL and the XRCE-DDS agent are started on their own (spec A.3, tabs 1
and 2); this file replaces tabs 3 to 5 and adds the display. In
mode explore (default) the aircraft takes off by itself once PX4, the
energy signal and the executor are ready, explores and lands on a
registry zone. mode:=observe only computes and displays.

The static transform map -> map_ned is the rotation of 180 degrees about
(1, 1, 0) / sqrt(2), quaternion (0.7071, 0.7071, 0, 0): it maps NED
(x north, y east, z down) onto the ENU display frame (x east, y north,
z up), so NED topics such as /cloud_world appear in the same scene.

    ros2 launch uav_core exploration.launch.py
    ros2 launch uav_core exploration.launch.py policy:=c1_gain
    ros2 launch uav_core exploration.launch.py mode:=observe rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Bridge, static frame, cloud_world, manager and RViz."""
    share = get_package_share_directory('uav_core')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='explore'),
        DeclareLaunchArgument('policy', default_value='c1_c2'),
        DeclareLaunchArgument('alpha', default_value='0.5'),
        DeclareLaunchArgument('reserve_fraction', default_value='0.1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slider', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('capacity_j', default_value='200000.0'),
        # Exploration area, a square of half side area_half_m around the
        # take-off point, inside a map grid of grid_size_m. Defaults: the
        # 60 x 60 m validation world in the 100 m grid. A larger area,
        # e.g. grid_size_m:=200.0 area_half_m:=95.0, makes the battery
        # rather than the world the limit of the exploration.
        DeclareLaunchArgument('area_half_m', default_value='30.0'),
        DeclareLaunchArgument('grid_size_m', default_value='100.0'),
        DeclareLaunchArgument('log_root',
                              default_value='~/ros2_ws/droneAfter/missions'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/lidar/points@sensor_msgs/msg/PointCloud2'
                        '[gz.msgs.PointCloudPacked',
                        '/x500_slz_0/command/motor_speed'
                        '@actuator_msgs/msg/Actuators[gz.msgs.Actuators',
                        # simulation clock, for cloud_world to take the
                        # pose at the scan stamp (pose_buffer)
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             condition=IfCondition(LaunchConfiguration('bridge')),
             output='screen'),
        Node(package='uav_core', executable='energy_monitor',
             parameters=[{'capacity_j': ParameterValue(
                 LaunchConfiguration('capacity_j'), value_type=float)}],
             output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--qx', '0.70710678', '--qy', '0.70710678',
                        '--qz', '0', '--qw', '0',
                        '--frame-id', 'map', '--child-frame-id', 'map_ned']),
        Node(package='uav_core', executable='cloud_world',
             parameters=[{'grid_size': ParameterValue(
                 LaunchConfiguration('grid_size_m'), value_type=float)}],
             output='screen'),
        Node(package='uav_core', executable='trajectory_executor',
             output='screen'),
        Node(package='uav_core', executable='exploration_manager',
             parameters=[{'mode': LaunchConfiguration('mode'),
                          'policy': LaunchConfiguration('policy'),
                          'alpha': ParameterValue(
                              LaunchConfiguration('alpha'), value_type=float),
                          'reserve_fraction': ParameterValue(
                              LaunchConfiguration('reserve_fraction'),
                              value_type=float),
                          'area_half_m': ParameterValue(
                              LaunchConfiguration('area_half_m'),
                              value_type=float),
                          'log_root': LaunchConfiguration('log_root')}],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz',
                                           'exploration.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        # Slider for alpha in flight: rqt_reconfigure shows one for any
        # parameter declared with a range; pick exploration_manager.
        Node(package='rqt_reconfigure', executable='rqt_reconfigure',
             condition=IfCondition(LaunchConfiguration('slider'))),
    ])
