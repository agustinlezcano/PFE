// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice

#ifndef EXTRA_INTERFACES__MSG__DETAIL__TRAMA__TRAITS_HPP_
#define EXTRA_INTERFACES__MSG__DETAIL__TRAMA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "extra_interfaces/msg/detail/trama__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace extra_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Trama & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Trama & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Trama & msg, bool use_flow_style = false)
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

}  // namespace extra_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use extra_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const extra_interfaces::msg::Trama & msg,
  std::ostream & out, size_t indentation = 0)
{
  extra_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use extra_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const extra_interfaces::msg::Trama & msg)
{
  return extra_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<extra_interfaces::msg::Trama>()
{
  return "extra_interfaces::msg::Trama";
}

template<>
inline const char * name<extra_interfaces::msg::Trama>()
{
  return "extra_interfaces/msg/Trama";
}

template<>
struct has_fixed_size<extra_interfaces::msg::Trama>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<extra_interfaces::msg::Trama>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<extra_interfaces::msg::Trama>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EXTRA_INTERFACES__MSG__DETAIL__TRAMA__TRAITS_HPP_
