// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'r_center'
// Member 'target_center'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__Rune __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__Rune __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Rune_
{
  using Type = Rune_<ContainerAllocator>;

  explicit Rune_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    r_center(_init),
    target_center(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rune_mode = 0l;
      this->real_angle = 0.0f;
      this->radian = 0.0f;
      this->rows = 0l;
      this->cols = 0l;
    }
  }

  explicit Rune_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    r_center(_alloc, _init),
    target_center(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rune_mode = 0l;
      this->real_angle = 0.0f;
      this->radian = 0.0f;
      this->rows = 0l;
      this->cols = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rune_mode_type =
    int32_t;
  _rune_mode_type rune_mode;
  using _r_center_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _r_center_type r_center;
  using _target_center_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_center_type target_center;
  using _real_angle_type =
    float;
  _real_angle_type real_angle;
  using _radian_type =
    float;
  _radian_type radian;
  using _rows_type =
    int32_t;
  _rows_type rows;
  using _cols_type =
    int32_t;
  _cols_type cols;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rune_mode(
    const int32_t & _arg)
  {
    this->rune_mode = _arg;
    return *this;
  }
  Type & set__r_center(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->r_center = _arg;
    return *this;
  }
  Type & set__target_center(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_center = _arg;
    return *this;
  }
  Type & set__real_angle(
    const float & _arg)
  {
    this->real_angle = _arg;
    return *this;
  }
  Type & set__radian(
    const float & _arg)
  {
    this->radian = _arg;
    return *this;
  }
  Type & set__rows(
    const int32_t & _arg)
  {
    this->rows = _arg;
    return *this;
  }
  Type & set__cols(
    const int32_t & _arg)
  {
    this->cols = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::Rune_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::Rune_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Rune_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Rune_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__Rune
    std::shared_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__Rune
    std::shared_ptr<auto_aim_interfaces::msg::Rune_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Rune_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rune_mode != other.rune_mode) {
      return false;
    }
    if (this->r_center != other.r_center) {
      return false;
    }
    if (this->target_center != other.target_center) {
      return false;
    }
    if (this->real_angle != other.real_angle) {
      return false;
    }
    if (this->radian != other.radian) {
      return false;
    }
    if (this->rows != other.rows) {
      return false;
    }
    if (this->cols != other.cols) {
      return false;
    }
    return true;
  }
  bool operator!=(const Rune_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Rune_

// alias to use template instance with default allocator
using Rune =
  auto_aim_interfaces::msg::Rune_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__STRUCT_HPP_
