// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/DebugRune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/DebugRune in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__DebugRune
{
  std_msgs__msg__Header header;
  float spin_speed;
  double filter_speed;
  double c_function;
} auto_aim_interfaces__msg__DebugRune;

// Struct for a sequence of auto_aim_interfaces__msg__DebugRune.
typedef struct auto_aim_interfaces__msg__DebugRune__Sequence
{
  auto_aim_interfaces__msg__DebugRune * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__DebugRune__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__STRUCT_H_
