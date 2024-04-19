// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/rune__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `r_center`
// Member `target_center`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
auto_aim_interfaces__msg__Rune__init(auto_aim_interfaces__msg__Rune * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    auto_aim_interfaces__msg__Rune__fini(msg);
    return false;
  }
  // rune_mode
  // r_center
  if (!geometry_msgs__msg__Point__init(&msg->r_center)) {
    auto_aim_interfaces__msg__Rune__fini(msg);
    return false;
  }
  // target_center
  if (!geometry_msgs__msg__Point__init(&msg->target_center)) {
    auto_aim_interfaces__msg__Rune__fini(msg);
    return false;
  }
  // real_angle
  // radian
  // rows
  // cols
  return true;
}

void
auto_aim_interfaces__msg__Rune__fini(auto_aim_interfaces__msg__Rune * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rune_mode
  // r_center
  geometry_msgs__msg__Point__fini(&msg->r_center);
  // target_center
  geometry_msgs__msg__Point__fini(&msg->target_center);
  // real_angle
  // radian
  // rows
  // cols
}

bool
auto_aim_interfaces__msg__Rune__are_equal(const auto_aim_interfaces__msg__Rune * lhs, const auto_aim_interfaces__msg__Rune * rhs)
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
  // rune_mode
  if (lhs->rune_mode != rhs->rune_mode) {
    return false;
  }
  // r_center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->r_center), &(rhs->r_center)))
  {
    return false;
  }
  // target_center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_center), &(rhs->target_center)))
  {
    return false;
  }
  // real_angle
  if (lhs->real_angle != rhs->real_angle) {
    return false;
  }
  // radian
  if (lhs->radian != rhs->radian) {
    return false;
  }
  // rows
  if (lhs->rows != rhs->rows) {
    return false;
  }
  // cols
  if (lhs->cols != rhs->cols) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__Rune__copy(
  const auto_aim_interfaces__msg__Rune * input,
  auto_aim_interfaces__msg__Rune * output)
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
  // rune_mode
  output->rune_mode = input->rune_mode;
  // r_center
  if (!geometry_msgs__msg__Point__copy(
      &(input->r_center), &(output->r_center)))
  {
    return false;
  }
  // target_center
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_center), &(output->target_center)))
  {
    return false;
  }
  // real_angle
  output->real_angle = input->real_angle;
  // radian
  output->radian = input->radian;
  // rows
  output->rows = input->rows;
  // cols
  output->cols = input->cols;
  return true;
}

auto_aim_interfaces__msg__Rune *
auto_aim_interfaces__msg__Rune__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Rune * msg = (auto_aim_interfaces__msg__Rune *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Rune), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__Rune));
  bool success = auto_aim_interfaces__msg__Rune__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__Rune__destroy(auto_aim_interfaces__msg__Rune * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__Rune__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__Rune__Sequence__init(auto_aim_interfaces__msg__Rune__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Rune * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__Rune *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__Rune), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__Rune__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__Rune__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__Rune__Sequence__fini(auto_aim_interfaces__msg__Rune__Sequence * array)
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
      auto_aim_interfaces__msg__Rune__fini(&array->data[i]);
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

auto_aim_interfaces__msg__Rune__Sequence *
auto_aim_interfaces__msg__Rune__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Rune__Sequence * array = (auto_aim_interfaces__msg__Rune__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Rune__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__Rune__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__Rune__Sequence__destroy(auto_aim_interfaces__msg__Rune__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__Rune__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__Rune__Sequence__are_equal(const auto_aim_interfaces__msg__Rune__Sequence * lhs, const auto_aim_interfaces__msg__Rune__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__Rune__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__Rune__Sequence__copy(
  const auto_aim_interfaces__msg__Rune__Sequence * input,
  auto_aim_interfaces__msg__Rune__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__Rune);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__Rune * data =
      (auto_aim_interfaces__msg__Rune *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__Rune__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__Rune__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__Rune__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
