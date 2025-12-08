// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice

#ifndef EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_H_
#define EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Trama in the package extra_interfaces.
typedef struct extra_interfaces__msg__Trama
{
  rosidl_runtime_c__String data;
} extra_interfaces__msg__Trama;

// Struct for a sequence of extra_interfaces__msg__Trama.
typedef struct extra_interfaces__msg__Trama__Sequence
{
  extra_interfaces__msg__Trama * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} extra_interfaces__msg__Trama__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_H_
