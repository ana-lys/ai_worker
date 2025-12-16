// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ffw_collision_checker/msg/detail/collision_check__functions.h"
#include "ffw_collision_checker/msg/detail/collision_check__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ffw_collision_checker
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CollisionCheck_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ffw_collision_checker::msg::CollisionCheck(_init);
}

void CollisionCheck_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ffw_collision_checker::msg::CollisionCheck *>(message_memory);
  typed_message->~CollisionCheck();
}

size_t size_function__CollisionCheck__distances(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CollisionCheck__distances(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__CollisionCheck__distances(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__CollisionCheck__distances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__CollisionCheck__distances(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__CollisionCheck__distances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__CollisionCheck__distances(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__CollisionCheck__distances(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CollisionCheck__geom1_names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CollisionCheck__geom1_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__CollisionCheck__geom1_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__CollisionCheck__geom1_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__CollisionCheck__geom1_names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__CollisionCheck__geom1_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__CollisionCheck__geom1_names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__CollisionCheck__geom1_names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CollisionCheck__geom2_names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CollisionCheck__geom2_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__CollisionCheck__geom2_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__CollisionCheck__geom2_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__CollisionCheck__geom2_names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__CollisionCheck__geom2_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__CollisionCheck__geom2_names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__CollisionCheck__geom2_names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CollisionCheck_message_member_array[4] = {
  {
    "in_collision",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ffw_collision_checker::msg::CollisionCheck, in_collision),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "distances",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ffw_collision_checker::msg::CollisionCheck, distances),  // bytes offset in struct
    nullptr,  // default value
    size_function__CollisionCheck__distances,  // size() function pointer
    get_const_function__CollisionCheck__distances,  // get_const(index) function pointer
    get_function__CollisionCheck__distances,  // get(index) function pointer
    fetch_function__CollisionCheck__distances,  // fetch(index, &value) function pointer
    assign_function__CollisionCheck__distances,  // assign(index, value) function pointer
    resize_function__CollisionCheck__distances  // resize(index) function pointer
  },
  {
    "geom1_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ffw_collision_checker::msg::CollisionCheck, geom1_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__CollisionCheck__geom1_names,  // size() function pointer
    get_const_function__CollisionCheck__geom1_names,  // get_const(index) function pointer
    get_function__CollisionCheck__geom1_names,  // get(index) function pointer
    fetch_function__CollisionCheck__geom1_names,  // fetch(index, &value) function pointer
    assign_function__CollisionCheck__geom1_names,  // assign(index, value) function pointer
    resize_function__CollisionCheck__geom1_names  // resize(index) function pointer
  },
  {
    "geom2_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ffw_collision_checker::msg::CollisionCheck, geom2_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__CollisionCheck__geom2_names,  // size() function pointer
    get_const_function__CollisionCheck__geom2_names,  // get_const(index) function pointer
    get_function__CollisionCheck__geom2_names,  // get(index) function pointer
    fetch_function__CollisionCheck__geom2_names,  // fetch(index, &value) function pointer
    assign_function__CollisionCheck__geom2_names,  // assign(index, value) function pointer
    resize_function__CollisionCheck__geom2_names  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CollisionCheck_message_members = {
  "ffw_collision_checker::msg",  // message namespace
  "CollisionCheck",  // message name
  4,  // number of fields
  sizeof(ffw_collision_checker::msg::CollisionCheck),
  false,  // has_any_key_member_
  CollisionCheck_message_member_array,  // message members
  CollisionCheck_init_function,  // function to initialize message memory (memory has to be allocated)
  CollisionCheck_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CollisionCheck_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CollisionCheck_message_members,
  get_message_typesupport_handle_function,
  &ffw_collision_checker__msg__CollisionCheck__get_type_hash,
  &ffw_collision_checker__msg__CollisionCheck__get_type_description,
  &ffw_collision_checker__msg__CollisionCheck__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ffw_collision_checker


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ffw_collision_checker::msg::CollisionCheck>()
{
  return &::ffw_collision_checker::msg::rosidl_typesupport_introspection_cpp::CollisionCheck_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ffw_collision_checker, msg, CollisionCheck)() {
  return &::ffw_collision_checker::msg::rosidl_typesupport_introspection_cpp::CollisionCheck_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
