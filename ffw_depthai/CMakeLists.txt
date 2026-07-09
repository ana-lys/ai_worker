cmake_minimum_required(VERSION 3.8)
project(ffw_depthai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Point CMake to where we installed depthai-core standalone (inside this package)
set(DEPTHAI_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/depthai_install")
list(APPEND CMAKE_PREFIX_PATH "${DEPTHAI_INSTALL_DIR}")
set(depthai_DIR "${DEPTHAI_INSTALL_DIR}/lib/cmake/depthai")

# Check if depthai is already installed
find_package(depthai QUIET)

if(NOT depthai_FOUND)
    message(STATUS "=======================================================================")
    message(STATUS " depthai-core not found in ${DEPTHAI_INSTALL_DIR}.")
    message(STATUS " Automatically running install_depthai.sh to build it.")
    message(STATUS " This may take 1-3 minutes. Please be patient...")
    message(STATUS "=======================================================================")
    
    execute_process(
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/install_depthai.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE INSTALL_RESULT
    )
    
    if(NOT INSTALL_RESULT EQUAL 0)
        message(FATAL_ERROR "Automatic installation failed! Please manually run:\n bash ${CMAKE_CURRENT_SOURCE_DIR}/install_depthai.sh\nThen re-run colcon build.")
    endif()
    
    # Try finding it again now that it's installed
    find_package(depthai REQUIRED)
endif()

# Build our dummy ROS node
add_executable(depthai_dummy_node src/depthai_node.cpp)
target_include_directories(depthai_dummy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(depthai_dummy_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  depthai_dummy_node
  "rclcpp"
)
# Link the core depthai library
target_link_libraries(depthai_dummy_node depthai::core)

# Install the ROS node
install(TARGETS depthai_dummy_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally, copy the depthai shared libraries so ROS can find them at runtime
install(DIRECTORY ${DEPTHAI_INSTALL_DIR}/lib/ DESTINATION lib/)

ament_package()
