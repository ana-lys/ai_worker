cmake_minimum_required(VERSION 3.16)
project(ffw_collision_checker)

# ---------------------------------------------------------------------------
# Run MuJoCo installer / Update Checker
# ---------------------------------------------------------------------------
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/mujoco)
set(STAMP_FILE ${MUJOCO_DIR}/.installed)

message(STATUS "[MuJoCo] Checking for updates...")

# We execute the script every time. The script itself handles the logic 
# of whether to download (if new version exists) or skip (if up to date).
execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/install_mujoco.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party
    RESULT_VARIABLE MUJOCO_INSTALL_STATUS
)

if(NOT MUJOCO_INSTALL_STATUS EQUAL 0)
    message(FATAL_ERROR "[MuJoCo] Installation or update failed.")
endif()

# Read the version string produced by the script for the install(FILES) step
if(EXISTS ${STAMP_FILE})
    file(READ ${STAMP_FILE} MUJOCO_VERSION_STR)
    string(STRIP "${MUJOCO_VERSION_STR}" MUJOCO_VERSION_STR)
else()
    set(MUJOCO_VERSION_STR "3.0.0") # Generic fallback
endif()

# ---------------------------------------------------------------------------
# Add MuJoCo paths
# ---------------------------------------------------------------------------
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARY ${MUJOCO_DIR}/lib/libmujoco.so)

message(STATUS "[MuJoCo] Include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "[MuJoCo] Library: ${MUJOCO_LIBRARY} (Version: ${MUJOCO_VERSION_STR})")

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
# Build nodes
# ---------------------------------------------------------------------------
# Common function to avoid code duplication for multiple executables
function(setup_checker_target TARGET_NAME)
    add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
    
    target_include_directories(${TARGET_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${MUJOCO_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIRS}
    )

    ament_target_dependencies(${TARGET_NAME}
        rclcpp
        sensor_msgs
        ament_index_cpp
    )

    # Link service typesupport
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    
    target_link_libraries(${TARGET_NAME}
        ${cpp_typesupport_target}
        mujoco
        ${GLFW_LIBRARIES}
        OpenGL::GL
    )
endfunction()

setup_checker_target(collision_checker)
setup_checker_target(replay)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

# Install MuJoCo libraries with dynamic versioning
install(FILES
    ${MUJOCO_DIR}/lib/libmujoco.so
    ${MUJOCO_DIR}/lib/libmujoco.so.${MUJOCO_VERSION_STR}
    DESTINATION lib
)

# Install 3rd party resources (XMLs, models)
install(DIRECTORY 3rd_party
    DESTINATION share/${PROJECT_NAME}
)

# Install built binaries
install(TARGETS collision_checker replay
    DESTINATION lib/${PROJECT_NAME}
)

# Set RPATH so binaries can find libmujoco.so in the workspace 'lib' folder
set_target_properties(collision_checker replay PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../lib:$ORIGIN/.."
)

ament_package()
