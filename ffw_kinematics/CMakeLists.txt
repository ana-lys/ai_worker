cmake_minimum_required(VERSION 3.8)
project(ffw_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# ffw Arm IK Solver executable
add_executable(ffw_arm_ik_solver src/ffw_arm_ik_solver.cpp)

target_include_directories(ffw_arm_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
)


target_compile_features(ffw_arm_ik_solver PUBLIC c_std_99 cxx_std_17)

target_link_libraries(ffw_arm_ik_solver
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  kdl_parser::kdl_parser
  urdf::urdf
)

# ffw Arm Trajectory Commander executable
add_executable(ffw_arm_trajectory_commander src/ffw_arm_trajectory_commander.cpp)

target_include_directories(ffw_arm_trajectory_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
)

target_compile_features(ffw_arm_trajectory_commander PUBLIC c_std_99 cxx_std_17)

target_link_libraries(ffw_arm_trajectory_commander
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
)


# Install C++ executables
install(TARGETS ffw_arm_ik_solver ffw_arm_trajectory_commander
  DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
