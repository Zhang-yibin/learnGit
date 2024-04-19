// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TargetRune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_RUNE__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_RUNE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/target_rune__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetRune_pitch_diff
{
public:
  explicit Init_TargetRune_pitch_diff(::auto_aim_interfaces::msg::TargetRune & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TargetRune pitch_diff(::auto_aim_interfaces::msg::TargetRune::_pitch_diff_type arg)
  {
    msg_.pitch_diff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetRune msg_;
};

class Init_TargetRune_yaw_diff
{
public:
  explicit Init_TargetRune_yaw_diff(::auto_aim_interfaces::msg::TargetRune & msg)
  : msg_(msg)
  {}
  Init_TargetRune_pitch_diff yaw_diff(::auto_aim_interfaces::msg::TargetRune::_yaw_diff_type arg)
  {
    msg_.yaw_diff = std::move(arg);
    return Init_TargetRune_pitch_diff(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetRune msg_;
};

class Init_TargetRune_yaw
{
public:
  explicit Init_TargetRune_yaw(::auto_aim_interfaces::msg::TargetRune & msg)
  : msg_(msg)
  {}
  Init_TargetRune_yaw_diff yaw(::auto_aim_interfaces::msg::TargetRune::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_TargetRune_yaw_diff(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetRune msg_;
};

class Init_TargetRune_pitch
{
public:
  explicit Init_TargetRune_pitch(::auto_aim_interfaces::msg::TargetRune & msg)
  : msg_(msg)
  {}
  Init_TargetRune_yaw pitch(::auto_aim_interfaces::msg::TargetRune::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_TargetRune_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetRune msg_;
};

class Init_TargetRune_header
{
public:
  Init_TargetRune_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetRune_pitch header(::auto_aim_interfaces::msg::TargetRune::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TargetRune_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TargetRune msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TargetRune>()
{
  return auto_aim_interfaces::msg::builder::Init_TargetRune_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET_RUNE__BUILDER_HPP_
