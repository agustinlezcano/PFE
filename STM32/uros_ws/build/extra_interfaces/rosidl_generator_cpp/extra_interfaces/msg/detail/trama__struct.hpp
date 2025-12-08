// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice

#ifndef EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_HPP_
#define EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__extra_interfaces__msg__Trama __attribute__((deprecated))
#else
# define DEPRECATED__extra_interfaces__msg__Trama __declspec(deprecated)
#endif

namespace extra_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Trama_
{
  using Type = Trama_<ContainerAllocator>;

  explicit Trama_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit Trama_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    extra_interfaces::msg::Trama_<ContainerAllocator> *;
  using ConstRawPtr =
    const extra_interfaces::msg::Trama_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<extra_interfaces::msg::Trama_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<extra_interfaces::msg::Trama_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      extra_interfaces::msg::Trama_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<extra_interfaces::msg::Trama_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      extra_interfaces::msg::Trama_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<extra_interfaces::msg::Trama_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<extra_interfaces::msg::Trama_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<extra_interfaces::msg::Trama_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__extra_interfaces__msg__Trama
    std::shared_ptr<extra_interfaces::msg::Trama_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__extra_interfaces__msg__Trama
    std::shared_ptr<extra_interfaces::msg::Trama_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Trama_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Trama_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Trama_

// alias to use template instance with default allocator
using Trama =
  extra_interfaces::msg::Trama_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace extra_interfaces

#endif  // EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_HPP_
