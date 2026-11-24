cmake_minimum_required(VERSION 3.8)
project(antbot_mpc_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/publish_test_path.py
  DESTINATION lib/${PROJECT_NAME} RENAME publish_test_path)
ament_package()
