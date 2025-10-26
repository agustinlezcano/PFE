// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice

#ifndef EXTRA_INTERFACES__MSG__DETAIL__TRAMA__BUILDER_HPP_
#define EXTRA_INTERFACES__MSG__DETAIL__TRAMA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "extra_interfaces/msg/detail/trama__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace extra_interfaces
{

namespace msg
{

namespace builder
{

class Init_Trama_data
{
public:
  Init_Trama_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::extra_interfaces::msg::Trama data(::extra_interfaces::msg::Trama::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::extra_interfaces::msg::Trama msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::extra_interfaces::msg::Trama>()
{
  return extra_interfaces::msg::builder::Init_Trama_data();
}

}  // namespace extra_interfaces

#endif  // EXTRA_INTERFACES__MSG__DETAIL__TRAMA__BUILDER_HPP_
