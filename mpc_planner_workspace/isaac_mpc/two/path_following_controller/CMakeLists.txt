cmake_minimum_required(VERSION 3.5)
project(antbot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(antbot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED)

add_executable(mpc_rpp_controller src/mpc_rpp_controller.cpp)
target_compile_features(mpc_rpp_controller PRIVATE cxx_std_17)
target_include_directories(mpc_rpp_controller PRIVATE ${Boost_INCLUDE_DIRS})
ament_target_dependencies(mpc_rpp_controller rclcpp rclcpp_lifecycle lifecycle_msgs nav2_core
  nav2_costmap_2d nav2_util pluginlib antbot_interfaces nav_msgs geometry_msgs std_msgs std_srvs)
install(TARGETS mpc_rpp_controller DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/scan_fix_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
