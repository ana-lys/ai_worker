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

# ────────────────────────────────────────────────────────────────
# RealSense streamer (all platforms)
# ────────────────────────────────────────────────────────────────
add_executable(realsense_udp_streamer src/realsense_udp_streamer.cpp)
target_include_directories(realsense_udp_streamer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(realsense_udp_streamer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(realsense_udp_streamer rclcpp)
target_link_libraries(realsense_udp_streamer
  realsense2::realsense2
  pthread
)
install(TARGETS realsense_udp_streamer
  DESTINATION lib/${PROJECT_NAME})

# ────────────────────────────────────────────────────────────────
# UDP receiver (all platforms)
# ────────────────────────────────────────────────────────────────
add_executable(realsense_udp_receiver src/realsense_udp_receiver.cpp)
target_include_directories(realsense_udp_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(realsense_udp_receiver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(realsense_udp_receiver rclcpp sensor_msgs cv_bridge)
target_link_libraries(realsense_udp_receiver ${OpenCV_LIBS})
install(TARGETS realsense_udp_receiver
  DESTINATION lib/${PROJECT_NAME})

# ────────────────────────────────────────────────────────────────
# ZED streamer — aarch64 (Jetson) ONLY
# Uses shared-library sandwich to bypass libnvargus linker bug
# ────────────────────────────────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  find_package(ZED QUIET)
  find_package(CUDA QUIET)

  if(ZED_FOUND AND CUDA_FOUND)
    message(STATUS "[ZED] aarch64 detected — building zed_udp_streamer")
    message(STATUS "[ZED] SDK: ${ZED_INCLUDE_DIRS}")
    message(STATUS "[ZED] CUDA: ${CUDA_INCLUDE_DIRS}")

    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

    link_directories(${ZED_LIBRARY_DIR})
    link_directories(${CUDA_LIBRARY_DIRS})

    # ── Step 1: Core logic as SHARED lib (relaxed linker — bypasses nvargus bug)
    add_library(zed_udp_streamer_core SHARED
      src/zed_udp_streamer.cpp
    )
    target_include_directories(zed_udp_streamer_core PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${ZED_INCLUDE_DIRS}
      ${CUDA_INCLUDE_DIRS}
    )
    if(EXISTS "/usr/local/cuda/include")
      target_include_directories(zed_udp_streamer_core PUBLIC /usr/local/cuda/include)
    endif()

    target_compile_features(zed_udp_streamer_core PUBLIC c_std_99 cxx_std_17)

    target_link_libraries(zed_udp_streamer_core
      ${ZED_LIBRARIES}
      ${CUDA_LIBRARIES}
      pthread
    )

    # Tegra JPEG — link into .so only (where linker is relaxed)
    if(EXISTS "/usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so")
      message(STATUS "[ZED] Linking Tegra libnvjpeg into shared lib")
      target_link_libraries(zed_udp_streamer_core
        /usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so
      )
    endif()

    # Allow unresolved symbols in the .so (safety net for nvargus bug)
    target_link_options(zed_udp_streamer_core PRIVATE
      -Wl,--allow-shlib-undefined
    )

    # ── Step 2: Tiny executable — only links OUR .so, never touches ZED directly
    add_executable(zed_udp_streamer src/zed_udp_streamer_main.cpp)
    target_include_directories(zed_udp_streamer PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      # NO ZED_INCLUDE_DIRS here — only your own header
    )
    target_compile_features(zed_udp_streamer PUBLIC c_std_99 cxx_std_17)
    target_link_libraries(zed_udp_streamer
      zed_udp_streamer_core  # only our .so
      pthread
    )

    install(TARGETS zed_udp_streamer DESTINATION lib/${PROJECT_NAME})
    install(TARGETS zed_udp_streamer_core DESTINATION lib)

  else()
    message(WARNING "[ZED] aarch64 detected but ZED SDK or CUDA not found — skipping zed_udp_streamer")
  endif()

else()
  message(STATUS "[ZED] Not aarch64 (${CMAKE_SYSTEM_PROCESSOR}) — skipping ZED streamer")
endif()

# ────────────────────────────────────────────────────────────────
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()