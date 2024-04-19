// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/DebugSendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__STRUCT_H_

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
#include "auto_aim_interfaces/msg/detail/debug_send__struct.h"

/// Struct defined in msg/DebugSendArmors in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__DebugSendArmors
{
  float center_angle;
  auto_aim_interfaces__msg__DebugSend__Sequence data;
} auto_aim_interfaces__msg__DebugSendArmors;

// Struct for a sequence of auto_aim_interfaces__msg__DebugSendArmors.
typedef struct auto_aim_interfaces__msg__DebugSendArmors__Sequence
{
  auto_aim_interfaces__msg__DebugSendArmors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__DebugSendArmors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__STRUCT_H_
