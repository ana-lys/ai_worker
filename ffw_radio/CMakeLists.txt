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
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ffw_collision_checker exports ffw_ik_solver_lib with proxsuite/mujoco in its link interface.
# Make those targets available before find_package(ffw_collision_checker).
set(FFW_COLLISION_CHECKER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ffw_collision_checker)
set(PROXSUITE_CONFIG_DIR ${FFW_COLLISION_CHECKER_SRC_DIR}/3rd_party/proxsuite/install/lib/cmake/proxsuite)
find_package(proxsuite CONFIG REQUIRED PATHS ${PROXSUITE_CONFIG_DIR} NO_DEFAULT_PATH)

if(NOT TARGET proxsuite::proxsuite-vectorized AND TARGET proxsuite::proxsuite)
  add_library(proxsuite::proxsuite-vectorized INTERFACE IMPORTED)
  target_link_libraries(proxsuite::proxsuite-vectorized INTERFACE proxsuite::proxsuite)
endif()

if(NOT TARGET mujoco)
  add_library(mujoco SHARED IMPORTED GLOBAL)
  set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION ${FFW_COLLISION_CHECKER_SRC_DIR}/3rd_party/mujoco/lib/libmujoco.so
    INTERFACE_INCLUDE_DIRECTORIES ${FFW_COLLISION_CHECKER_SRC_DIR}/3rd_party/mujoco/include
  )
endif()

find_package(ffw_collision_checker REQUIRED)

# Build the Dashboard C++ node
add_executable(radio_dashboard src/radio_dashboard.cpp)
ament_target_dependencies(radio_dashboard rclcpp sensor_msgs joy)

# Build the control mapper C++ node
add_executable(radio_control_mapper src/radio_control_mapper.cpp)
ament_target_dependencies(radio_control_mapper rclcpp sensor_msgs geometry_msgs trajectory_msgs joy ament_index_cpp ffw_collision_checker)
target_link_libraries(radio_control_mapper Eigen3::Eigen)
if(TARGET ffw_collision_checker::ffw_ik_solver_lib)
  target_link_libraries(radio_control_mapper ffw_collision_checker::ffw_ik_solver_lib)
else()
  target_link_libraries(radio_control_mapper ffw_ik_solver_lib)
endif()

# Install C++ executable
install(TARGETS
  radio_dashboard
  radio_control_mapper
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
