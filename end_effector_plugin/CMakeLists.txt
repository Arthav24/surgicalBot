cmake_minimum_required(VERSION 3.5)
project(end_effector_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)  # Adjust the version as necessary
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include Ignition Gazebo directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${IGNITION_GAZEBO_INCLUDE_DIRS}  # Include Ignition Gazebo directories
)

add_library(EndEffectorPositionPlugin SHARED src/EndEffectorPositionPlugin.cc)

ament_target_dependencies(EndEffectorPositionPlugin
  rclcpp
  geometry_msgs
  ignition-gazebo6  # Adjust the version as necessary
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  EndEffectorPositionPlugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()