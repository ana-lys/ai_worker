// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/msg/collision_check.hpp"


#ifndef FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__BUILDER_HPP_
#define FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ffw_collision_checker/msg/detail/collision_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ffw_collision_checker
{

namespace msg
{

namespace builder
{

class Init_CollisionCheck_geom2_names
{
public:
  explicit Init_CollisionCheck_geom2_names(::ffw_collision_checker::msg::CollisionCheck & msg)
  : msg_(msg)
  {}
  ::ffw_collision_checker::msg::CollisionCheck geom2_names(::ffw_collision_checker::msg::CollisionCheck::_geom2_names_type arg)
  {
    msg_.geom2_names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ffw_collision_checker::msg::CollisionCheck msg_;
};

class Init_CollisionCheck_geom1_names
{
public:
  explicit Init_CollisionCheck_geom1_names(::ffw_collision_checker::msg::CollisionCheck & msg)
  : msg_(msg)
  {}
  Init_CollisionCheck_geom2_names geom1_names(::ffw_collision_checker::msg::CollisionCheck::_geom1_names_type arg)
  {
    msg_.geom1_names = std::move(arg);
    return Init_CollisionCheck_geom2_names(msg_);
  }

private:
  ::ffw_collision_checker::msg::CollisionCheck msg_;
};

class Init_CollisionCheck_distances
{
public:
  explicit Init_CollisionCheck_distances(::ffw_collision_checker::msg::CollisionCheck & msg)
  : msg_(msg)
  {}
  Init_CollisionCheck_geom1_names distances(::ffw_collision_checker::msg::CollisionCheck::_distances_type arg)
  {
    msg_.distances = std::move(arg);
    return Init_CollisionCheck_geom1_names(msg_);
  }

private:
  ::ffw_collision_checker::msg::CollisionCheck msg_;
};

class Init_CollisionCheck_in_collision
{
public:
  Init_CollisionCheck_in_collision()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CollisionCheck_distances in_collision(::ffw_collision_checker::msg::CollisionCheck::_in_collision_type arg)
  {
    msg_.in_collision = std::move(arg);
    return Init_CollisionCheck_distances(msg_);
  }

private:
  ::ffw_collision_checker::msg::CollisionCheck msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ffw_collision_checker::msg::CollisionCheck>()
{
  return ffw_collision_checker::msg::builder::Init_CollisionCheck_in_collision();
}

}  // namespace ffw_collision_checker

#endif  // FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__BUILDER_HPP_
