cmake_minimum_required(VERSION 3.16)
project(ffw_collision_checker)

# ---------------------------------------------------------------------------
# Run MuJoCo installer (only if libs missing)
# ---------------------------------------------------------------------------
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mujoco)
set(MUJOCO_LIB ${MUJOCO_DIR}/lib/libmujoco.so)
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
# ROS 2 dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveCollisionNaive.srv"
  "msg/CollisionCheck.msg"
)

# ---------------------------------------------------------------------------
# Build node
# ---------------------------------------------------------------------------
add_executable(collision_checker src/collision_checker.cpp)
add_executable(replay src/replay.cpp)

target_include_directories(collision_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIRS}
)
ament_target_dependencies(collision_checker
  rclcpp
  sensor_msgs
  ament_index_cpp
)

# Link service typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(collision_checker
  ${cpp_typesupport_target}
  mujoco            # equivalent to libmujoco.so
  ${GLFW_LIBRARIES}
  OpenGL::GL
)

target_include_directories(replay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIRS}
)
ament_target_dependencies(replay
  rclcpp
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(replay
  mujoco
  ${GLFW_LIBRARIES}
  OpenGL::GL
)

# Install MuJoCo libraries
install(FILES
  ${MUJOCO_DIR}/lib/libmujoco.so
  ${MUJOCO_DIR}/lib/libmujoco.so.3.4.0
  DESTINATION lib
)

# Install 3rd party resources (XMLs)
install(DIRECTORY 3rd_party
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS collision_checker replay
  DESTINATION lib/${PROJECT_NAME})

# Set RPATH to find the library in the install/lib folder
set_target_properties(collision_checker replay PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../lib:$ORIGIN/.."
)

ament_package()
