// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/msg/collision_check.hpp"


#ifndef FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__TRAITS_HPP_
#define FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ffw_collision_checker/msg/detail/collision_check__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ffw_collision_checker
{

namespace msg
{

inline void to_flow_style_yaml(
  const CollisionCheck & msg,
  std::ostream & out)
{
  out << "{";
  // member: in_collision
  {
    out << "in_collision: ";
    rosidl_generator_traits::value_to_yaml(msg.in_collision, out);
    out << ", ";
  }

  // member: distances
  {
    if (msg.distances.size() == 0) {
      out << "distances: []";
    } else {
      out << "distances: [";
      size_t pending_items = msg.distances.size();
      for (auto item : msg.distances) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: geom1_names
  {
    if (msg.geom1_names.size() == 0) {
      out << "geom1_names: []";
    } else {
      out << "geom1_names: [";
      size_t pending_items = msg.geom1_names.size();
      for (auto item : msg.geom1_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: geom2_names
  {
    if (msg.geom2_names.size() == 0) {
      out << "geom2_names: []";
    } else {
      out << "geom2_names: [";
      size_t pending_items = msg.geom2_names.size();
      for (auto item : msg.geom2_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CollisionCheck & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: in_collision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_collision: ";
    rosidl_generator_traits::value_to_yaml(msg.in_collision, out);
    out << "\n";
  }

  // member: distances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.distances.size() == 0) {
      out << "distances: []\n";
    } else {
      out << "distances:\n";
      for (auto item : msg.distances) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: geom1_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.geom1_names.size() == 0) {
      out << "geom1_names: []\n";
    } else {
      out << "geom1_names:\n";
      for (auto item : msg.geom1_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: geom2_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.geom2_names.size() == 0) {
      out << "geom2_names: []\n";
    } else {
      out << "geom2_names:\n";
      for (auto item : msg.geom2_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CollisionCheck & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ffw_collision_checker

namespace rosidl_generator_traits
{

[[deprecated("use ffw_collision_checker::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ffw_collision_checker::msg::CollisionCheck & msg,
  std::ostream & out, size_t indentation = 0)
{
  ffw_collision_checker::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ffw_collision_checker::msg::to_yaml() instead")]]
inline std::string to_yaml(const ffw_collision_checker::msg::CollisionCheck & msg)
{
  return ffw_collision_checker::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ffw_collision_checker::msg::CollisionCheck>()
{
  return "ffw_collision_checker::msg::CollisionCheck";
}

template<>
inline const char * name<ffw_collision_checker::msg::CollisionCheck>()
{
  return "ffw_collision_checker/msg/CollisionCheck";
}

template<>
struct has_fixed_size<ffw_collision_checker::msg::CollisionCheck>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ffw_collision_checker::msg::CollisionCheck>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ffw_collision_checker::msg::CollisionCheck>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__TRAITS_HPP_
