cmake_minimum_required(VERSION 3.8)
project(ffw_depthai)

# C++17 & warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# DepthAI (your custom install logic)
set(DEPTHAI_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/depthai_install")
list(APPEND CMAKE_PREFIX_PATH "${DEPTHAI_INSTALL_DIR}")
set(depthai_DIR "${DEPTHAI_INSTALL_DIR}/lib/cmake/depthai")

find_package(depthai QUIET)
if(NOT depthai_FOUND)
    message(STATUS "depthai-core not found. Running install script...")
    execute_process(
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/install_depthai.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE INSTALL_RESULT
    )
    if(NOT INSTALL_RESULT EQUAL 0)
        message(FATAL_ERROR "Automatic depthai installation failed")
    endif()
    find_package(depthai REQUIRED)
endif()

# ---------- GStreamer ----------
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# Build the node
add_executable(depthai_udp_streamer src/depthai_node.cpp)
target_include_directories(depthai_udp_streamer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GSTREAMER_INCLUDE_DIRS}
)
target_compile_features(depthai_udp_streamer PUBLIC cxx_std_17)

ament_target_dependencies(depthai_udp_streamer rclcpp)

target_link_libraries(depthai_udp_streamer
    depthai::core
    ${GSTREAMER_LIBRARIES}
)

# Install
install(TARGETS depthai_udp_streamer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${DEPTHAI_INSTALL_DIR}/lib/ DESTINATION lib/)

ament_package()