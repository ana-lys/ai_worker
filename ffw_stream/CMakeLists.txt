cmake_minimum_required(VERSION 3.8)
project(ffw_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZED QUIET)

add_executable(realsense_udp_streamer src/realsense_udp_streamer.cpp)
target_include_directories(realsense_udp_streamer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(realsense_udp_streamer PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realsense_udp_streamer
  rclcpp
)

target_link_libraries(realsense_udp_streamer
  realsense2::realsense2
  pthread
)

install(TARGETS realsense_udp_streamer
  DESTINATION lib/${PROJECT_NAME})

if (ZED_FOUND)
  add_executable(zed_udp_streamer src/zed_udp_streamer.cpp)
  target_include_directories(zed_udp_streamer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ZED_INCLUDE_DIRS}
  )
  if (EXISTS "/usr/local/cuda/include")
    target_include_directories(zed_udp_streamer PUBLIC /usr/local/cuda/include)
  endif()
  
  target_compile_features(zed_udp_streamer PUBLIC c_std_99 cxx_std_17)

  target_link_libraries(zed_udp_streamer
    ${ZED_LIBRARIES}
    pthread
  )
  
  find_library(NVJPEG_LIB nvjpeg)
  if (NVJPEG_LIB)
    target_link_libraries(zed_udp_streamer ${NVJPEG_LIB})
  endif()

  install(TARGETS zed_udp_streamer
    DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(realsense_udp_receiver src/realsense_udp_receiver.cpp)
target_include_directories(realsense_udp_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(realsense_udp_receiver PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(realsense_udp_receiver
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(realsense_udp_receiver
  ${OpenCV_LIBS}
)

install(TARGETS realsense_udp_receiver
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()