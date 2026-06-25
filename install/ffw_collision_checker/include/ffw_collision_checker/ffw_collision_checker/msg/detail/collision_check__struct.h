// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/msg/collision_check.h"


#ifndef FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__STRUCT_H_
#define FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'distances'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'geom1_names'
// Member 'geom2_names'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CollisionCheck in the package ffw_collision_checker.
/**
  * CollisionCheck.msg
  * Message for publishing collision check results
 */
typedef struct ffw_collision_checker__msg__CollisionCheck
{
  /// Whether any collision is detected (distance < 0)
  bool in_collision;
  /// Array of distances for all contact pairs
  /// Negative values indicate penetration depth
  rosidl_runtime_c__double__Sequence distances;
  /// Names of first geometry in each contact pair
  rosidl_runtime_c__String__Sequence geom1_names;
  /// Names of second geometry in each contact pair
  rosidl_runtime_c__String__Sequence geom2_names;
} ffw_collision_checker__msg__CollisionCheck;

// Struct for a sequence of ffw_collision_checker__msg__CollisionCheck.
typedef struct ffw_collision_checker__msg__CollisionCheck__Sequence
{
  ffw_collision_checker__msg__CollisionCheck * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ffw_collision_checker__msg__CollisionCheck__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__STRUCT_H_
