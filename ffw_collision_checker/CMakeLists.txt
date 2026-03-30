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
# Run proxsuite installer / Update Checker
# ---------------------------------------------------------------------------
set(PROXSUITE_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/proxsuite)
set(PROXSUITE_INSTALL_DIR ${PROXSUITE_SRC_DIR}/install)
set(PROXSUITE_STAMP_FILE ${PROXSUITE_SRC_DIR}/.installed)

add_custom_target(proxsuite_sync ALL
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/install_proxsuite.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party
    COMMENT "[proxsuite] Syncing dependency (clone/update/build/install if needed)"
)

if(EXISTS ${PROXSUITE_STAMP_FILE})
    file(READ ${PROXSUITE_STAMP_FILE} PROXSUITE_HASH_STR)
    string(STRIP "${PROXSUITE_HASH_STR}" PROXSUITE_HASH_STR)
else()
    set(PROXSUITE_HASH_STR "unknown")
endif()

# ---------------------------------------------------------------------------
# Add MuJoCo paths
# ---------------------------------------------------------------------------
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARY ${MUJOCO_DIR}/lib/libmujoco.so)
set(PROXSUITE_INCLUDE_DIR ${PROXSUITE_INSTALL_DIR}/include)
set(PROXSUITE_LIBRARY_DIR ${PROXSUITE_INSTALL_DIR}/lib)

message(STATUS "[MuJoCo] Include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "[MuJoCo] Library: ${MUJOCO_LIBRARY} (Version: ${MUJOCO_VERSION_STR})")
message(STATUS "[proxsuite] Include: ${PROXSUITE_INCLUDE_DIR}")
message(STATUS "[proxsuite] Library dir: ${PROXSUITE_LIBRARY_DIR} (Commit: ${PROXSUITE_HASH_STR})")

include_directories(${MUJOCO_INCLUDE_DIR})
include_directories(${PROXSUITE_INCLUDE_DIR})
link_directories(${MUJOCO_DIR}/lib)
link_directories(${PROXSUITE_LIBRARY_DIR})

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
find_package(Boost REQUIRED)
find_package(coal REQUIRED)
find_package(pinocchio REQUIRED)

find_package(proxsuite CONFIG REQUIRED
    PATHS ${PROXSUITE_INSTALL_DIR}/lib/cmake/proxsuite
    NO_DEFAULT_PATH
)

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

add_dependencies(collision_checker proxsuite_sync)
add_dependencies(replay proxsuite_sync)

add_executable(dependency_versions_example src/proxsuite_example.cpp)
target_compile_definitions(dependency_versions_example PRIVATE MUJOCO_STAMP_VERSION="${MUJOCO_VERSION_STR}")
target_include_directories(dependency_versions_example PRIVATE
    ${MUJOCO_INCLUDE_DIR}
    ${coal_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)
target_link_libraries(dependency_versions_example PUBLIC proxsuite::proxsuite mujoco)
add_dependencies(dependency_versions_example proxsuite_sync)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

# Install MuJoCo libraries with dynamic versioning
install(FILES
    ${MUJOCO_DIR}/lib/libmujoco.so
    ${MUJOCO_DIR}/lib/libmujoco.so.${MUJOCO_VERSION_STR}
    DESTINATION lib
)

# Install proxsuite shared libraries if present
file(GLOB PROXSUITE_SHARED_LIBS
    ${PROXSUITE_LIBRARY_DIR}/*.so
    ${PROXSUITE_LIBRARY_DIR}/*.so.*
)

if(PROXSUITE_SHARED_LIBS)
    install(FILES
        ${PROXSUITE_SHARED_LIBS}
        DESTINATION lib
    )
endif()

# Install 3rd party resources (XMLs, models)
install(DIRECTORY 3rd_party
    DESTINATION share/${PROJECT_NAME}
)

# Install built binaries
install(TARGETS collision_checker replay dependency_versions_example
    DESTINATION lib/${PROJECT_NAME}
)

# Set RPATH so binaries can find libmujoco.so in the workspace 'lib' folder
set_target_properties(collision_checker replay PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../lib:$ORIGIN/.."
)

ament_package()
