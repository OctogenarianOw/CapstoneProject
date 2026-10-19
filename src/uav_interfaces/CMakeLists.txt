cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ElevationMap.msg"
  "msg/EnergyState.msg"
  "msg/ExecutorCommand.msg"
  "msg/ExecutorStatus.msg"
  "msg/PathCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
