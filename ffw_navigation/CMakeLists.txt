cmake_minimum_required(VERSION 3.8)
project(ffw_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(is_path_length_under_plugin SHARED
  src/is_path_length_under.cpp
)

target_include_directories(is_path_length_under_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(is_path_length_under_plugin
  behaviortree_cpp
  nav_msgs
  geometry_msgs
  rclcpp
)

install(TARGETS is_path_length_under_plugin
  EXPORT export_is_path_length_under_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  rviz
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

target_compile_definitions(is_path_length_under_plugin PRIVATE BT_PLUGIN_EXPORT)

ament_export_libraries(is_path_length_under_plugin)
ament_export_dependencies(behaviortree_cpp nav_msgs geometry_msgs rclcpp)

ament_package()
