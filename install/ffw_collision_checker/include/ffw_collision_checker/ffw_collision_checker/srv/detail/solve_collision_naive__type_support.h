// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from ffw_collision_checker:srv/SolveCollisionNaive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/srv/solve_collision_naive.h"


#ifndef FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__TYPE_SUPPORT_H_
#define FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "ffw_collision_checker/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  ffw_collision_checker,
  srv,
  SolveCollisionNaive_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  ffw_collision_checker,
  srv,
  SolveCollisionNaive_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  ffw_collision_checker,
  srv,
  SolveCollisionNaive_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  ffw_collision_checker,
  srv,
  SolveCollisionNaive
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  ffw_collision_checker,
  srv,
  SolveCollisionNaive
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  ffw_collision_checker,
  srv,
  SolveCollisionNaive
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__TYPE_SUPPORT_H_
