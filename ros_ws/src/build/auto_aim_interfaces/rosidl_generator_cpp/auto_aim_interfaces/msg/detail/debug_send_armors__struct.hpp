// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/DebugSendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'data'
#include "auto_aim_interfaces/msg/detail/debug_send__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__DebugSendArmors __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__DebugSendArmors __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DebugSendArmors_
{
  using Type = DebugSendArmors_<ContainerAllocator>;

  explicit DebugSendArmors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_angle = 0.0f;
    }
  }

  explicit DebugSendArmors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_angle = 0.0f;
    }
  }

  // field types and members
  using _center_angle_type =
    float;
  _center_angle_type center_angle;
  using _data_type =
    std::vector<auto_aim_interfaces::msg::DebugSend_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<auto_aim_interfaces::msg::DebugSend_<ContainerAllocator>>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__center_angle(
    const float & _arg)
  {
    this->center_angle = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<auto_aim_interfaces::msg::DebugSend_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<auto_aim_interfaces::msg::DebugSend_<ContainerAllocator>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__DebugSendArmors
    std::shared_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__DebugSendArmors
    std::shared_ptr<auto_aim_interfaces::msg::DebugSendArmors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DebugSendArmors_ & other) const
  {
    if (this->center_angle != other.center_angle) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DebugSendArmors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DebugSendArmors_

// alias to use template instance with default allocator
using DebugSendArmors =
  auto_aim_interfaces::msg::DebugSendArmors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__STRUCT_HPP_
