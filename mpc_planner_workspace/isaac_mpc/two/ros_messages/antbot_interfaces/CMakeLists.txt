cmake_minimum_required(VERSION 3.8)
project(antbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PeerState.msg"
  "msg/MpcPlan.msg"
  "msg/JointCommandFeedback.msg"
  DEPENDENCIES std_msgs nav_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_package()
