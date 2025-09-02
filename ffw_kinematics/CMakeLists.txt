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
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)



# Target Pose Publisher executable
add_executable(target_pose_publisher src/target_pose_publisher.cpp)

target_include_directories(target_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(target_pose_publisher PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(target_pose_publisher
  rclcpp
  geometry_msgs
)

# Realtime IK Solver with Joint Limits executable
add_executable(realtime_ik_solver_jl src/realtime_ik_solver_jl.cpp)

target_include_directories(realtime_ik_solver_jl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(realtime_ik_solver_jl PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realtime_ik_solver_jl
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  urdf
)

# Dual Arm IK Solver executable
add_executable(dual_arm_ik_solver src/dual_arm_ik_solver.cpp)

target_include_directories(dual_arm_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dual_arm_ik_solver PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(dual_arm_ik_solver
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  kdl_parser
  urdf
)

# Dual Arm Trajectory Commander executable
add_executable(dual_arm_trajectory_commander src/dual_arm_trajectory_commander.cpp)

target_include_directories(dual_arm_trajectory_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dual_arm_trajectory_commander PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(dual_arm_trajectory_commander
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

# Pinocchio IK Solver executable
add_executable(pinocchio_ik_solver src/pinocchio_ik_solver.cpp)

target_include_directories(pinocchio_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pinocchio_ik_solver PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(pinocchio_ik_solver
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  urdf
)


# VR Hand Pose Transformer executable
add_executable(vr_hand_pose_transformer src/vr_hand_pose_transformer.cpp)

target_include_directories(vr_hand_pose_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(vr_hand_pose_transformer PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(vr_hand_pose_transformer
  rclcpp
  geometry_msgs
)

# Link Pinocchio manually (since it might not have proper CMake integration in ROS2)
target_link_libraries(pinocchio_ik_solver pinocchio::pinocchio)

# Link Eigen3 to VR hand pose transformer
target_link_libraries(vr_hand_pose_transformer Eigen3::Eigen)

# Install C++ executables
install(TARGETS target_pose_publisher realtime_ik_solver_jl dual_arm_ik_solver dual_arm_trajectory_commander pinocchio_ik_solver vr_hand_pose_transformer
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
