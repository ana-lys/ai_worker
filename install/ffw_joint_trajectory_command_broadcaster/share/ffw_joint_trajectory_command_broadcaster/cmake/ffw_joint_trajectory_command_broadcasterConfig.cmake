# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ffw_joint_trajectory_command_broadcaster_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ffw_joint_trajectory_command_broadcaster_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ffw_joint_trajectory_command_broadcaster_FOUND FALSE)
  elseif(NOT ffw_joint_trajectory_command_broadcaster_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ffw_joint_trajectory_command_broadcaster_FOUND FALSE)
  endif()
  return()
endif()
set(_ffw_joint_trajectory_command_broadcaster_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ffw_joint_trajectory_command_broadcaster_FIND_QUIETLY)
  message(STATUS "Found ffw_joint_trajectory_command_broadcaster: 1.1.15 (${ffw_joint_trajectory_command_broadcaster_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ffw_joint_trajectory_command_broadcaster' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ffw_joint_trajectory_command_broadcaster_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ffw_joint_trajectory_command_broadcaster_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${ffw_joint_trajectory_command_broadcaster_DIR}/${_extra}")
endforeach()
