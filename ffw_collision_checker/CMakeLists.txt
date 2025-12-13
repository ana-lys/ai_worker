cmake_minimum_required(VERSION 3.16)
project(ffw_collision_checker)

# ---------------------------------------------------------------------------
# Run MuJoCo installer (only if libs missing)
# ---------------------------------------------------------------------------
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mujoco)
set(MUJOCO_LIB ${MUJOCO_DIR}/lib/libmujoco.so)
message(INFO " DEBUG: MUJOCO_HOME is set — failing intentionally")
if(NOT EXISTS ${MUJOCO_LIB})
  message(INFO "MuJoCo not found, running install script...")
  execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/install_mujoco.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party
    RESULT_VARIABLE MUJOCO_INSTALL_STATUS
  )
  if(NOT MUJOCO_INSTALL_STATUS EQUAL 0)
    message(FATAL_ERROR "MuJoCo installation failed.")
  endif()
else()
  message(INFO "MuJoCo already installed.")
endif()

# ---------------------------------------------------------------------------
# Add MuJoCo paths
# ---------------------------------------------------------------------------
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARY ${MUJOCO_DIR}/lib/libmujoco.so)

message(INFO "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(INFO "MuJoCo library: ${MUJOCO_LIBRARY}")

include_directories(${MUJOCO_INCLUDE_DIR})
link_directories(${MUJOCO_DIR}/lib)

# ---------------------------------------------------------------------------
# ROS 2 and MoveIt dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckCollision.srv"
)

# ---------------------------------------------------------------------------
# Build node
# ---------------------------------------------------------------------------
add_executable(collision_checker src/collision_checker.cpp)

target_include_directories(collision_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)

ament_target_dependencies(collision_checker
  rclcpp
  moveit_core
  moveit_ros_planning
)

# Link service typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(collision_checker
  ${cpp_typesupport_target}
  mujoco            # equivalent to libmujoco.so
)

install(TARGETS collision_checker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
