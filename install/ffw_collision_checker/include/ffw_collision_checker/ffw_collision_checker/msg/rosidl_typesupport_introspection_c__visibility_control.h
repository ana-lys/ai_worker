// generated from
// rosidl_typesupport_introspection_c/resource/rosidl_typesupport_introspection_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef FFW_COLLISION_CHECKER__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_
#define FFW_COLLISION_CHECKER__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ffw_collision_checker __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_ffw_collision_checker __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ffw_collision_checker __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_ffw_collision_checker __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_INTROSPECTION_C_BUILDING_DLL_ffw_collision_checker
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_ffw_collision_checker ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ffw_collision_checker
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_ffw_collision_checker ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_ffw_collision_checker
  #endif
#else
  #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ffw_collision_checker __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_ffw_collision_checker
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_ffw_collision_checker __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_ffw_collision_checker
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // FFW_COLLISION_CHECKER__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_
