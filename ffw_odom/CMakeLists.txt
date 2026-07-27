cmake_minimum_required(VERSION 3.8)
project(ffw_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Release-mode optimization matters for the ICP matching loop; without it
# the KD-tree/Eigen-heavy code can run several times slower.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(rf2o_laser_odometry_node
  src/CLaserOdometry2DNode.cpp
  src/CLaserOdometry2D.cpp
)

target_include_directories(rf2o_laser_odometry_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(rf2o_laser_odometry_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
)


add_executable(scan_to_map_icp
  src/ScanToMapICP.cpp
)

target_include_directories(scan_to_map_icp PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(scan_to_map_icp
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
)

add_executable(relocalize_test_node
  src/RelocalizeTestNode.cpp
)

target_include_directories(relocalize_test_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(relocalize_test_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
)

add_executable(scan_projector
  src/ScanProjector.cpp
)

target_include_directories(scan_projector PRIVATE
  include
)

ament_target_dependencies(scan_projector
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
)

install(TARGETS rf2o_laser_odometry_node scan_to_map_icp relocalize_test_node scan_projector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install the Python debug script so it is available via "ros2 run ffw_odom debug_scan_map.py".
install(PROGRAMS scripts/debug_scan_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()