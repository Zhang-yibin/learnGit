// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/DebugRune.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/debug_rune__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
auto_aim_interfaces__msg__DebugRune__init(auto_aim_interfaces__msg__DebugRune * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    auto_aim_interfaces__msg__DebugRune__fini(msg);
    return false;
  }
  // spin_speed
  // filter_speed
  // c_function
  return true;
}

void
auto_aim_interfaces__msg__DebugRune__fini(auto_aim_interfaces__msg__DebugRune * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // spin_speed
  // filter_speed
  // c_function
}

bool
auto_aim_interfaces__msg__DebugRune__are_equal(const auto_aim_interfaces__msg__DebugRune * lhs, const auto_aim_interfaces__msg__DebugRune * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // spin_speed
  if (lhs->spin_speed != rhs->spin_speed) {
    return false;
  }
  // filter_speed
  if (lhs->filter_speed != rhs->filter_speed) {
    return false;
  }
  // c_function
  if (lhs->c_function != rhs->c_function) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__DebugRune__copy(
  const auto_aim_interfaces__msg__DebugRune * input,
  auto_aim_interfaces__msg__DebugRune * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // spin_speed
  output->spin_speed = input->spin_speed;
  // filter_speed
  output->filter_speed = input->filter_speed;
  // c_function
  output->c_function = input->c_function;
  return true;
}

auto_aim_interfaces__msg__DebugRune *
auto_aim_interfaces__msg__DebugRune__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__DebugRune * msg = (auto_aim_interfaces__msg__DebugRune *)allocator.allocate(sizeof(auto_aim_interfaces__msg__DebugRune), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__DebugRune));
  bool success = auto_aim_interfaces__msg__DebugRune__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__DebugRune__destroy(auto_aim_interfaces__msg__DebugRune * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__DebugRune__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__DebugRune__Sequence__init(auto_aim_interfaces__msg__DebugRune__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__DebugRune * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__DebugRune *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__DebugRune), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__DebugRune__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__DebugRune__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
auto_aim_interfaces__msg__DebugRune__Sequence__fini(auto_aim_interfaces__msg__DebugRune__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      auto_aim_interfaces__msg__DebugRune__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

auto_aim_interfaces__msg__DebugRune__Sequence *
auto_aim_interfaces__msg__DebugRune__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__DebugRune__Sequence * array = (auto_aim_interfaces__msg__DebugRune__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__DebugRune__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__DebugRune__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__DebugRune__Sequence__destroy(auto_aim_interfaces__msg__DebugRune__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__DebugRune__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__DebugRune__Sequence__are_equal(const auto_aim_interfaces__msg__DebugRune__Sequence * lhs, const auto_aim_interfaces__msg__DebugRune__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__DebugRune__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__DebugRune__Sequence__copy(
  const auto_aim_interfaces__msg__DebugRune__Sequence * input,
  auto_aim_interfaces__msg__DebugRune__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__DebugRune);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__DebugRune * data =
      (auto_aim_interfaces__msg__DebugRune *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__DebugRune__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__DebugRune__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__DebugRune__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
