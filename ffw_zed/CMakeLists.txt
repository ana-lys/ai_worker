cmake_minimum_required(VERSION 3.8)
project(ffw_zed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)

find_package(ZED QUIET)
find_package(CUDA QUIET)
find_package(OpenCV QUIET)

if(ZED_FOUND AND CUDA_FOUND)
  set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

  link_directories(${ZED_LIBRARY_DIR})
  link_directories(${CUDA_LIBRARY_DIRS})

  add_library(ffw_zed_streamer_core SHARED
    src/ffw_zed_streamer.cpp
  )
  target_include_directories(ffw_zed_streamer_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  if(EXISTS "/usr/local/cuda/include")
    target_include_directories(ffw_zed_streamer_core PUBLIC /usr/local/cuda/include)
  endif()

  target_compile_features(ffw_zed_streamer_core PUBLIC c_std_99 cxx_std_17)

  target_link_libraries(ffw_zed_streamer_core
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
  )

  if(EXISTS "/usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so")
    target_link_libraries(ffw_zed_streamer_core /usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so)
  endif()

  target_link_options(ffw_zed_streamer_core PRIVATE -Wl,--allow-shlib-undefined)

  add_executable(ffw_zed_streamer src/ffw_zed_streamer_main.cpp)
  target_include_directories(ffw_zed_streamer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(ffw_zed_streamer PUBLIC c_std_99 cxx_std_17)
  target_link_libraries(ffw_zed_streamer
    ${CMAKE_DL_LIBS}
    pthread
  )

  add_custom_command(TARGET ffw_zed_streamer POST_BUILD
    COMMAND sudo setcap cap_sys_nice+eip $<TARGET_FILE:ffw_zed_streamer> || true
    COMMENT "Granting cap_sys_nice for SCHED_FIFO"
  )

  install(TARGETS ffw_zed_streamer DESTINATION lib/${PROJECT_NAME})
  install(TARGETS ffw_zed_streamer_core DESTINATION lib)
else()
  message(WARNING "[ffw_zed] ZED SDK or CUDA not found — skipping build")
endif()

ament_package()
