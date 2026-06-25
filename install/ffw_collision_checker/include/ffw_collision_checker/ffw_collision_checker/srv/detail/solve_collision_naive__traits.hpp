// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ffw_collision_checker:srv/SolveCollisionNaive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/srv/solve_collision_naive.hpp"


#ifndef FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__TRAITS_HPP_
#define FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ffw_collision_checker/srv/detail/solve_collision_naive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ffw_collision_checker
{

namespace srv
{

inline void to_flow_style_yaml(
  const SolveCollisionNaive_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_names
  {
    if (msg.joint_names.size() == 0) {
      out << "joint_names: []";
    } else {
      out << "joint_names: [";
      size_t pending_items = msg.joint_names.size();
      for (auto item : msg.joint_names) {
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
  const SolveCollisionNaive_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_names.size() == 0) {
      out << "joint_names: []\n";
    } else {
      out << "joint_names:\n";
      for (auto item : msg.joint_names) {
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

inline std::string to_yaml(const SolveCollisionNaive_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ffw_collision_checker

namespace rosidl_generator_traits
{

[[deprecated("use ffw_collision_checker::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ffw_collision_checker::srv::SolveCollisionNaive_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ffw_collision_checker::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ffw_collision_checker::srv::to_yaml() instead")]]
inline std::string to_yaml(const ffw_collision_checker::srv::SolveCollisionNaive_Request & msg)
{
  return ffw_collision_checker::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ffw_collision_checker::srv::SolveCollisionNaive_Request>()
{
  return "ffw_collision_checker::srv::SolveCollisionNaive_Request";
}

template<>
inline const char * name<ffw_collision_checker::srv::SolveCollisionNaive_Request>()
{
  return "ffw_collision_checker/srv/SolveCollisionNaive_Request";
}

template<>
struct has_fixed_size<ffw_collision_checker::srv::SolveCollisionNaive_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ffw_collision_checker::srv::SolveCollisionNaive_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ffw_collision_checker
{

namespace srv
{

inline void to_flow_style_yaml(
  const SolveCollisionNaive_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accept
  {
    out << "accept: ";
    rosidl_generator_traits::value_to_yaml(msg.accept, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SolveCollisionNaive_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accept
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accept: ";
    rosidl_generator_traits::value_to_yaml(msg.accept, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SolveCollisionNaive_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ffw_collision_checker

namespace rosidl_generator_traits
{

[[deprecated("use ffw_collision_checker::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ffw_collision_checker::srv::SolveCollisionNaive_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ffw_collision_checker::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ffw_collision_checker::srv::to_yaml() instead")]]
inline std::string to_yaml(const ffw_collision_checker::srv::SolveCollisionNaive_Response & msg)
{
  return ffw_collision_checker::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ffw_collision_checker::srv::SolveCollisionNaive_Response>()
{
  return "ffw_collision_checker::srv::SolveCollisionNaive_Response";
}

template<>
inline const char * name<ffw_collision_checker::srv::SolveCollisionNaive_Response>()
{
  return "ffw_collision_checker/srv/SolveCollisionNaive_Response";
}

template<>
struct has_fixed_size<ffw_collision_checker::srv::SolveCollisionNaive_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ffw_collision_checker::srv::SolveCollisionNaive_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace ffw_collision_checker
{

namespace srv
{

inline void to_flow_style_yaml(
  const SolveCollisionNaive_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
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
  const SolveCollisionNaive_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SolveCollisionNaive_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ffw_collision_checker

namespace rosidl_generator_traits
{

[[deprecated("use ffw_collision_checker::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ffw_collision_checker::srv::SolveCollisionNaive_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  ffw_collision_checker::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ffw_collision_checker::srv::to_yaml() instead")]]
inline std::string to_yaml(const ffw_collision_checker::srv::SolveCollisionNaive_Event & msg)
{
  return ffw_collision_checker::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ffw_collision_checker::srv::SolveCollisionNaive_Event>()
{
  return "ffw_collision_checker::srv::SolveCollisionNaive_Event";
}

template<>
inline const char * name<ffw_collision_checker::srv::SolveCollisionNaive_Event>()
{
  return "ffw_collision_checker/srv/SolveCollisionNaive_Event";
}

template<>
struct has_fixed_size<ffw_collision_checker::srv::SolveCollisionNaive_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Event>
  : std::integral_constant<bool, has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Request>::value && has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<ffw_collision_checker::srv::SolveCollisionNaive_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ffw_collision_checker::srv::SolveCollisionNaive>()
{
  return "ffw_collision_checker::srv::SolveCollisionNaive";
}

template<>
inline const char * name<ffw_collision_checker::srv::SolveCollisionNaive>()
{
  return "ffw_collision_checker/srv/SolveCollisionNaive";
}

template<>
struct has_fixed_size<ffw_collision_checker::srv::SolveCollisionNaive>
  : std::integral_constant<
    bool,
    has_fixed_size<ffw_collision_checker::srv::SolveCollisionNaive_Request>::value &&
    has_fixed_size<ffw_collision_checker::srv::SolveCollisionNaive_Response>::value
  >
{
};

template<>
struct has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive>
  : std::integral_constant<
    bool,
    has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Request>::value &&
    has_bounded_size<ffw_collision_checker::srv::SolveCollisionNaive_Response>::value
  >
{
};

template<>
struct is_service<ffw_collision_checker::srv::SolveCollisionNaive>
  : std::true_type
{
};

template<>
struct is_service_request<ffw_collision_checker::srv::SolveCollisionNaive_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ffw_collision_checker::srv::SolveCollisionNaive_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__TRAITS_HPP_
