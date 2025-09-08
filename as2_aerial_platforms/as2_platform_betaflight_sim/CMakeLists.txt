cmake_minimum_required(VERSION 3.5)
project(as2_platform_betaflight_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create executable
add_executable(betaflight_sim_node
  src/betaflight_sim_node.cpp
  src/betaflight_platform.cpp
)

# Dependencies
ament_target_dependencies(betaflight_sim_node
  rclcpp
  as2_core
  as2_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

# Install
install(TARGETS
  betaflight_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
