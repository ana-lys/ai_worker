// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice
#ifndef FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ffw_collision_checker/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ffw_collision_checker/msg/detail/collision_check__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
bool cdr_serialize_ffw_collision_checker__msg__CollisionCheck(
  const ffw_collision_checker__msg__CollisionCheck * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
bool cdr_deserialize_ffw_collision_checker__msg__CollisionCheck(
  eprosima::fastcdr::Cdr &,
  ffw_collision_checker__msg__CollisionCheck * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
size_t get_serialized_size_ffw_collision_checker__msg__CollisionCheck(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
size_t max_serialized_size_ffw_collision_checker__msg__CollisionCheck(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
bool cdr_serialize_key_ffw_collision_checker__msg__CollisionCheck(
  const ffw_collision_checker__msg__CollisionCheck * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
size_t get_serialized_size_key_ffw_collision_checker__msg__CollisionCheck(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
size_t max_serialized_size_key_ffw_collision_checker__msg__CollisionCheck(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ffw_collision_checker
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ffw_collision_checker, msg, CollisionCheck)();

#ifdef __cplusplus
}
#endif

#endif  // FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
