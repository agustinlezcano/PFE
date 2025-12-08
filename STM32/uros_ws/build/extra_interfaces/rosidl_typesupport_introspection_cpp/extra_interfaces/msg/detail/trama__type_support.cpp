// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "extra_interfaces/msg/detail/trama__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace extra_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Trama_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) extra_interfaces::msg::Trama(_init);
}

void Trama_fini_function(void * message_memory)
{
  auto typed_message = static_cast<extra_interfaces::msg::Trama *>(message_memory);
  typed_message->~Trama();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Trama_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(extra_interfaces::msg::Trama, data),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Trama_message_members = {
  "extra_interfaces::msg",  // message namespace
  "Trama",  // message name
  1,  // number of fields
  sizeof(extra_interfaces::msg::Trama),
  Trama_message_member_array,  // message members
  Trama_init_function,  // function to initialize message memory (memory has to be allocated)
  Trama_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Trama_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Trama_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace extra_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<extra_interfaces::msg::Trama>()
{
  return &::extra_interfaces::msg::rosidl_typesupport_introspection_cpp::Trama_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, extra_interfaces, msg, Trama)() {
  return &::extra_interfaces::msg::rosidl_typesupport_introspection_cpp::Trama_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
