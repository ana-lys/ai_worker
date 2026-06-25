# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:ffw_collision_checker__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:ffw_collision_checker__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:ffw_collision_checker__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:ffw_collision_checker__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:ffw_collision_checker__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:ffw_collision_checker__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:ffw_collision_checker__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:ffw_collision_checker__rosidl_typesupport_cpp;:ffw_collision_checker__rosidl_generator_py")

# populate ffw_collision_checker_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "ffw_collision_checker::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'ffw_collision_checker' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND ffw_collision_checker_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
