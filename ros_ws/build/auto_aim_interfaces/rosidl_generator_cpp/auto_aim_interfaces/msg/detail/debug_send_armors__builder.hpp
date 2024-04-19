// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/DebugSendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/debug_send_armors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_DebugSendArmors_data
{
public:
  explicit Init_DebugSendArmors_data(::auto_aim_interfaces::msg::DebugSendArmors & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::DebugSendArmors data(::auto_aim_interfaces::msg::DebugSendArmors::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugSendArmors msg_;
};

class Init_DebugSendArmors_center_angle
{
public:
  Init_DebugSendArmors_center_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugSendArmors_data center_angle(::auto_aim_interfaces::msg::DebugSendArmors::_center_angle_type arg)
  {
    msg_.center_angle = std::move(arg);
    return Init_DebugSendArmors_data(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugSendArmors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::DebugSendArmors>()
{
  return auto_aim_interfaces::msg::builder::Init_DebugSendArmors_center_angle();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND_ARMORS__BUILDER_HPP_
