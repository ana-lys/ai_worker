cmake_minimum_required(VERSION 3.16)
project(ffw_collision_checker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckCollision.srv"
)

# Build collision checker executable
add_executable(collision_checker src/collision_checker.cpp)
target_include_directories(collision_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(collision_checker
  rclcpp
  moveit_core
  moveit_ros_planning
)

# Link executable to generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(collision_checker "${cpp_typesupport_target}")

install(TARGETS collision_checker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
