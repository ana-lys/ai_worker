// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ffw_collision_checker:srv/SolveCollisionNaive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/srv/solve_collision_naive.hpp"


#ifndef FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__STRUCT_HPP_
#define FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Request __attribute__((deprecated))
#else
# define DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Request __declspec(deprecated)
#endif

namespace ffw_collision_checker
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SolveCollisionNaive_Request_
{
  using Type = SolveCollisionNaive_Request_<ContainerAllocator>;

  explicit SolveCollisionNaive_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SolveCollisionNaive_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _joint_names_type joint_names;

  // setters for named parameter idiom
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Request
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Request
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolveCollisionNaive_Request_ & other) const
  {
    if (this->joint_names != other.joint_names) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolveCollisionNaive_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolveCollisionNaive_Request_

// alias to use template instance with default allocator
using SolveCollisionNaive_Request =
  ffw_collision_checker::srv::SolveCollisionNaive_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ffw_collision_checker


#ifndef _WIN32
# define DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Response __attribute__((deprecated))
#else
# define DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Response __declspec(deprecated)
#endif

namespace ffw_collision_checker
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SolveCollisionNaive_Response_
{
  using Type = SolveCollisionNaive_Response_<ContainerAllocator>;

  explicit SolveCollisionNaive_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accept = false;
      this->error = 0l;
    }
  }

  explicit SolveCollisionNaive_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accept = false;
      this->error = 0l;
    }
  }

  // field types and members
  using _accept_type =
    bool;
  _accept_type accept;
  using _error_type =
    int32_t;
  _error_type error;

  // setters for named parameter idiom
  Type & set__accept(
    const bool & _arg)
  {
    this->accept = _arg;
    return *this;
  }
  Type & set__error(
    const int32_t & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Response
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Response
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolveCollisionNaive_Response_ & other) const
  {
    if (this->accept != other.accept) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolveCollisionNaive_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolveCollisionNaive_Response_

// alias to use template instance with default allocator
using SolveCollisionNaive_Response =
  ffw_collision_checker::srv::SolveCollisionNaive_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ffw_collision_checker


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Event __attribute__((deprecated))
#else
# define DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Event __declspec(deprecated)
#endif

namespace ffw_collision_checker
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SolveCollisionNaive_Event_
{
  using Type = SolveCollisionNaive_Event_<ContainerAllocator>;

  explicit SolveCollisionNaive_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SolveCollisionNaive_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ffw_collision_checker::srv::SolveCollisionNaive_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ffw_collision_checker::srv::SolveCollisionNaive_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Event
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ffw_collision_checker__srv__SolveCollisionNaive_Event
    std::shared_ptr<ffw_collision_checker::srv::SolveCollisionNaive_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolveCollisionNaive_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolveCollisionNaive_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolveCollisionNaive_Event_

// alias to use template instance with default allocator
using SolveCollisionNaive_Event =
  ffw_collision_checker::srv::SolveCollisionNaive_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ffw_collision_checker

namespace ffw_collision_checker
{

namespace srv
{

struct SolveCollisionNaive
{
  using Request = ffw_collision_checker::srv::SolveCollisionNaive_Request;
  using Response = ffw_collision_checker::srv::SolveCollisionNaive_Response;
  using Event = ffw_collision_checker::srv::SolveCollisionNaive_Event;
};

}  // namespace srv

}  // namespace ffw_collision_checker

#endif  // FFW_COLLISION_CHECKER__SRV__DETAIL__SOLVE_COLLISION_NAIVE__STRUCT_HPP_
