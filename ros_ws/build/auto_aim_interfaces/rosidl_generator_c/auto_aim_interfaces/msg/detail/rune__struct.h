// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__STRUCT_H_

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
// Member 'r_center'
// Member 'target_center'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Rune in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Rune
{
  std_msgs__msg__Header header;
  int32_t rune_mode;
  geometry_msgs__msg__Point r_center;
  geometry_msgs__msg__Point target_center;
  float real_angle;
  float radian;
  int32_t rows;
  int32_t cols;
} auto_aim_interfaces__msg__Rune;

// Struct for a sequence of auto_aim_interfaces__msg__Rune.
typedef struct auto_aim_interfaces__msg__Rune__Sequence
{
  auto_aim_interfaces__msg__Rune * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Rune__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__STRUCT_H_