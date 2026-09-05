cmake_minimum_required(VERSION 3.8)
project(ffw_il_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install the thin console launcher (overrides the setuptools entry-point
# wrapper, mirroring ffw_teleop: the installed file is a symlink to scripts/)
install(PROGRAMS
  scripts/il_episode_recorder
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
