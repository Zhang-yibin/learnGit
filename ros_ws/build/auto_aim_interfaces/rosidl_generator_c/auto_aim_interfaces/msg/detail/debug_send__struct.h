// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/DebugSend.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/DebugSend in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__DebugSend
{
  geometry_msgs__msg__Point point;
  float yaw;
  float angle;
  int32_t id;
} auto_aim_interfaces__msg__DebugSend;

// Struct for a sequence of auto_aim_interfaces__msg__DebugSend.
typedef struct auto_aim_interfaces__msg__DebugSend__Sequence
{
  auto_aim_interfaces__msg__DebugSend * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__DebugSend__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND__STRUCT_H_
