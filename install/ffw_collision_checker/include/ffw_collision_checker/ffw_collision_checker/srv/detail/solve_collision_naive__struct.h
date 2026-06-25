// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ffw_collision_checker:srv/SolveCollisionNaive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/srv/solve_collision_naive.h"


#ifndef FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__STRUCT_H_
#define FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_names'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SolveCollisionNaive in the package ffw_collision_checker.
typedef struct ffw_collision_checker__srv__SolveCollisionNaive_Request
{
  rosidl_runtime_c__String__Sequence joint_names;
} ffw_collision_checker__srv__SolveCollisionNaive_Request;

// Struct for a sequence of ffw_collision_checker__srv__SolveCollisionNaive_Request.
typedef struct ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence
{
  ffw_collision_checker__srv__SolveCollisionNaive_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SolveCollisionNaive in the package ffw_collision_checker.
typedef struct ffw_collision_checker__srv__SolveCollisionNaive_Response
{
  bool accept;
  int32_t error;
} ffw_collision_checker__srv__SolveCollisionNaive_Response;

// Struct for a sequence of ffw_collision_checker__srv__SolveCollisionNaive_Response.
typedef struct ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence
{
  ffw_collision_checker__srv__SolveCollisionNaive_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  ffw_collision_checker__srv__SolveCollisionNaive_Event__request__MAX_SIZE = 1
};
// response
enum
{
  ffw_collision_checker__srv__SolveCollisionNaive_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SolveCollisionNaive in the package ffw_collision_checker.
typedef struct ffw_collision_checker__srv__SolveCollisionNaive_Event
{
  service_msgs__msg__ServiceEventInfo info;
  ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence request;
  ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence response;
} ffw_collision_checker__srv__SolveCollisionNaive_Event;

// Struct for a sequence of ffw_collision_checker__srv__SolveCollisionNaive_Event.
typedef struct ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence
{
  ffw_collision_checker__srv__SolveCollisionNaive_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__STRUCT_H_
