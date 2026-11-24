cmake_minimum_required(VERSION 3.8)
project(gaemi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(gaemi_interfaces
  "msg/MonoCameraArray.msg"
  "msg/DepthCameraArray.msg"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
