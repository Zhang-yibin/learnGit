// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from auto_aim_interfaces:msg/TargetRune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_RUNE__FUNCTIONS_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_RUNE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "auto_aim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "auto_aim_interfaces/msg/detail/target_rune__struct.h"

/// Initialize msg/TargetRune message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * auto_aim_interfaces__msg__TargetRune
 * )) before or use
 * auto_aim_interfaces__msg__TargetRune__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TargetRune__init(auto_aim_interfaces__msg__TargetRune * msg);

/// Finalize msg/TargetRune message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TargetRune__fini(auto_aim_interfaces__msg__TargetRune * msg);

/// Create msg/TargetRune message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * auto_aim_interfaces__msg__TargetRune__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
auto_aim_interfaces__msg__TargetRune *
auto_aim_interfaces__msg__TargetRune__create();

/// Destroy msg/TargetRune message.
/**
 * It calls
 * auto_aim_interfaces__msg__TargetRune__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TargetRune__destroy(auto_aim_interfaces__msg__TargetRune * msg);

/// Check for msg/TargetRune message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TargetRune__are_equal(const auto_aim_interfaces__msg__TargetRune * lhs, const auto_aim_interfaces__msg__TargetRune * rhs);

/// Copy a msg/TargetRune message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TargetRune__copy(
  const auto_aim_interfaces__msg__TargetRune * input,
  auto_aim_interfaces__msg__TargetRune * output);

/// Initialize array of msg/TargetRune messages.
/**
 * It allocates the memory for the number of elements and calls
 * auto_aim_interfaces__msg__TargetRune__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TargetRune__Sequence__init(auto_aim_interfaces__msg__TargetRune__Sequence * array, size_t size);

/// Finalize array of msg/TargetRune messages.
/**
 * It calls
 * auto_aim_interfaces__msg__TargetRune__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TargetRune__Sequence__fini(auto_aim_interfaces__msg__TargetRune__Sequence * array);

/// Create array of msg/TargetRune messages.
/**
 * It allocates the memory for the array and calls
 * auto_aim_interfaces__msg__TargetRune__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
auto_aim_interfaces__msg__TargetRune__Sequence *
auto_aim_interfaces__msg__TargetRune__Sequence__create(size_t size);

/// Destroy array of msg/TargetRune messages.
/**
 * It calls
 * auto_aim_interfaces__msg__TargetRune__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__TargetRune__Sequence__destroy(auto_aim_interfaces__msg__TargetRune__Sequence * array);

/// Check for msg/TargetRune message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TargetRune__Sequence__are_equal(const auto_aim_interfaces__msg__TargetRune__Sequence * lhs, const auto_aim_interfaces__msg__TargetRune__Sequence * rhs);

/// Copy an array of msg/TargetRune messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__TargetRune__Sequence__copy(
  const auto_aim_interfaces__msg__TargetRune__Sequence * input,
  auto_aim_interfaces__msg__TargetRune__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_RUNE__FUNCTIONS_H_
