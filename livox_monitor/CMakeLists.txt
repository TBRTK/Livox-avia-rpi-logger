cmake_minimum_required(VERSION 3.8)
project(livox_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_interfaces REQUIRED)

add_executable(livox_monitor_node
  src/livox_monitor.cpp
)

ament_target_dependencies(
  livox_monitor_node
  rclcpp
  sensor_msgs
  livox_interfaces
)

install(
  TARGETS livox_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
