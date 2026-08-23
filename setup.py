from setuptools import find_packages, setup
package_name = 'uav_navigation'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dorian',
    maintainer_email='dorian@todo.todo',
    description='UAV navigation and terrain scoring for Capstone A',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_node = uav_navigation.waypoint_node:main',
            'terrain_scoring_node = uav_navigation.terrain_scoring_node:main',
            'lidar_processor_node = uav_navigation.lidar_processor_node:main',
            'obstacle_layer_node = uav_navigation.obstacle_layer_node:main',
            'terrain_follower_node = uav_navigation.terrain_follower_node:main',
            'frontier_detector_node = uav_navigation.frontier_detector_node:main',
            'astar_planner_node = uav_navigation.astar_planner_node:main',
            'path_watchdog_node = uav_navigation.path_watchdog_node:main',
            'exploration_manager_node = uav_navigation.exploration_manager_node:main',
        ],
    },
)
