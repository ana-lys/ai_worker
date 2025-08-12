cmake_minimum_required(VERSION 3.14)
project(ffw_vr_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_executable(vr_hand_control_node
  src/vr_hand_control_node.cpp
)

ament_target_dependencies(vr_hand_control_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  tf2_kdl
  kdl_parser
  orocos_kdl
  trac_ik_lib
  visualization_msgs
)

install(TARGETS
  vr_hand_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
