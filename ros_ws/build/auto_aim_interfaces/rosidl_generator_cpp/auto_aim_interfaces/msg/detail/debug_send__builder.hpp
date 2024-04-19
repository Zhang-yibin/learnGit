// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/DebugSend.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/debug_send__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_DebugSend_id
{
public:
  explicit Init_DebugSend_id(::auto_aim_interfaces::msg::DebugSend & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::DebugSend id(::auto_aim_interfaces::msg::DebugSend::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugSend msg_;
};

class Init_DebugSend_angle
{
public:
  explicit Init_DebugSend_angle(::auto_aim_interfaces::msg::DebugSend & msg)
  : msg_(msg)
  {}
  Init_DebugSend_id angle(::auto_aim_interfaces::msg::DebugSend::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_DebugSend_id(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugSend msg_;
};

class Init_DebugSend_yaw
{
public:
  explicit Init_DebugSend_yaw(::auto_aim_interfaces::msg::DebugSend & msg)
  : msg_(msg)
  {}
  Init_DebugSend_angle yaw(::auto_aim_interfaces::msg::DebugSend::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_DebugSend_angle(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugSend msg_;
};

class Init_DebugSend_point
{
public:
  Init_DebugSend_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugSend_yaw point(::auto_aim_interfaces::msg::DebugSend::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_DebugSend_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugSend msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::DebugSend>()
{
  return auto_aim_interfaces::msg::builder::Init_DebugSend_point();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_SEND__BUILDER_HPP_
