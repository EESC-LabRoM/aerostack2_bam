cmake_minimum_required(VERSION 3.5)
project(as2_platform_betaflight_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create executable
add_executable(as2_platform_betaflight_sim_node
  src/betaflight_sim_node.cpp
  src/betaflight_platform.cpp
  src/msp_helper.cpp
)

# Dependencies
ament_target_dependencies(as2_platform_betaflight_sim_node
  rclcpp
  as2_core
  as2_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  ros_gz_interfaces
)

# Install
install(TARGETS
  as2_platform_betaflight_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/betaflight_gazebo_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME betaflight_gazebo_tf_bridge
)

ament_package()
