// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ffw_collision_checker:srv/SolveCollisionNaive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/srv/solve_collision_naive.hpp"


#ifndef FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__BUILDER_HPP_
#define FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ffw_collision_checker/srv/detail/solve_collision_naive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ffw_collision_checker
{

namespace srv
{

namespace builder
{

class Init_SolveCollisionNaive_Request_joint_names
{
public:
  Init_SolveCollisionNaive_Request_joint_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ffw_collision_checker::srv::SolveCollisionNaive_Request joint_names(::ffw_collision_checker::srv::SolveCollisionNaive_Request::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ffw_collision_checker::srv::SolveCollisionNaive_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ffw_collision_checker::srv::SolveCollisionNaive_Request>()
{
  return ffw_collision_checker::srv::builder::Init_SolveCollisionNaive_Request_joint_names();
}

}  // namespace ffw_collision_checker


namespace ffw_collision_checker
{

namespace srv
{

namespace builder
{

class Init_SolveCollisionNaive_Response_error
{
public:
  explicit Init_SolveCollisionNaive_Response_error(::ffw_collision_checker::srv::SolveCollisionNaive_Response & msg)
  : msg_(msg)
  {}
  ::ffw_collision_checker::srv::SolveCollisionNaive_Response error(::ffw_collision_checker::srv::SolveCollisionNaive_Response::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ffw_collision_checker::srv::SolveCollisionNaive_Response msg_;
};

class Init_SolveCollisionNaive_Response_accept
{
public:
  Init_SolveCollisionNaive_Response_accept()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SolveCollisionNaive_Response_error accept(::ffw_collision_checker::srv::SolveCollisionNaive_Response::_accept_type arg)
  {
    msg_.accept = std::move(arg);
    return Init_SolveCollisionNaive_Response_error(msg_);
  }

private:
  ::ffw_collision_checker::srv::SolveCollisionNaive_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ffw_collision_checker::srv::SolveCollisionNaive_Response>()
{
  return ffw_collision_checker::srv::builder::Init_SolveCollisionNaive_Response_accept();
}

}  // namespace ffw_collision_checker


namespace ffw_collision_checker
{

namespace srv
{

namespace builder
{

class Init_SolveCollisionNaive_Event_response
{
public:
  explicit Init_SolveCollisionNaive_Event_response(::ffw_collision_checker::srv::SolveCollisionNaive_Event & msg)
  : msg_(msg)
  {}
  ::ffw_collision_checker::srv::SolveCollisionNaive_Event response(::ffw_collision_checker::srv::SolveCollisionNaive_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ffw_collision_checker::srv::SolveCollisionNaive_Event msg_;
};

class Init_SolveCollisionNaive_Event_request
{
public:
  explicit Init_SolveCollisionNaive_Event_request(::ffw_collision_checker::srv::SolveCollisionNaive_Event & msg)
  : msg_(msg)
  {}
  Init_SolveCollisionNaive_Event_response request(::ffw_collision_checker::srv::SolveCollisionNaive_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SolveCollisionNaive_Event_response(msg_);
  }

private:
  ::ffw_collision_checker::srv::SolveCollisionNaive_Event msg_;
};

class Init_SolveCollisionNaive_Event_info
{
public:
  Init_SolveCollisionNaive_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SolveCollisionNaive_Event_request info(::ffw_collision_checker::srv::SolveCollisionNaive_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SolveCollisionNaive_Event_request(msg_);
  }

private:
  ::ffw_collision_checker::srv::SolveCollisionNaive_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ffw_collision_checker::srv::SolveCollisionNaive_Event>()
{
  return ffw_collision_checker::srv::builder::Init_SolveCollisionNaive_Event_info();
}

}  // namespace ffw_collision_checker

#endif  // FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__BUILDER_HPP_
