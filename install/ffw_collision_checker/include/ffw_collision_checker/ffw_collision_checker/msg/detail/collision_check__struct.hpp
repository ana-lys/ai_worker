// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ffw_collision_checker/msg/collision_check.hpp"


#ifndef FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__STRUCT_HPP_
#define FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ffw_collision_checker__msg__CollisionCheck __attribute__((deprecated))
#else
# define DEPRECATED__ffw_collision_checker__msg__CollisionCheck __declspec(deprecated)
#endif

namespace ffw_collision_checker
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CollisionCheck_
{
  using Type = CollisionCheck_<ContainerAllocator>;

  explicit CollisionCheck_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->in_collision = false;
    }
  }

  explicit CollisionCheck_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->in_collision = false;
    }
  }

  // field types and members
  using _in_collision_type =
    bool;
  _in_collision_type in_collision;
  using _distances_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _distances_type distances;
  using _geom1_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _geom1_names_type geom1_names;
  using _geom2_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _geom2_names_type geom2_names;

  // setters for named parameter idiom
  Type & set__in_collision(
    const bool & _arg)
  {
    this->in_collision = _arg;
    return *this;
  }
  Type & set__distances(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->distances = _arg;
    return *this;
  }
  Type & set__geom1_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->geom1_names = _arg;
    return *this;
  }
  Type & set__geom2_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->geom2_names = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator> *;
  using ConstRawPtr =
    const ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ffw_collision_checker__msg__CollisionCheck
    std::shared_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ffw_collision_checker__msg__CollisionCheck
    std::shared_ptr<ffw_collision_checker::msg::CollisionCheck_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CollisionCheck_ & other) const
  {
    if (this->in_collision != other.in_collision) {
      return false;
    }
    if (this->distances != other.distances) {
      return false;
    }
    if (this->geom1_names != other.geom1_names) {
      return false;
    }
    if (this->geom2_names != other.geom2_names) {
      return false;
    }
    return true;
  }
  bool operator!=(const CollisionCheck_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CollisionCheck_

// alias to use template instance with default allocator
using CollisionCheck =
  ffw_collision_checker::msg::CollisionCheck_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ffw_collision_checker

#endif  // FFW_COLLISION_CHECKER__MSG__DETAIL__COLLISION_CHECK__STRUCT_HPP_
