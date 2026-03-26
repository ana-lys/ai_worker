cmake_minimum_required(VERSION 3.8)
project(ffw_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/keyboard_control
  scripts/keyboard_control_simulation
  scripts/trajectory_test
  scripts/mobile_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Build C++ node
add_executable(keyboard_swerve_drive src/keyboard_swerve_drive.cpp)
ament_target_dependencies(keyboard_swerve_drive rclcpp geometry_msgs)

install(TARGETS
  keyboard_swerve_drive
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install resource
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
