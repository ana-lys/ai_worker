cmake_minimum_required(VERSION 3.8)
project(ffw_radio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build the Dashboard C++ node
add_executable(radio_dashboard src/radio_dashboard.cpp)
ament_target_dependencies(radio_dashboard rclcpp sensor_msgs)

# Install C++ executable
install(TARGETS
  radio_dashboard
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