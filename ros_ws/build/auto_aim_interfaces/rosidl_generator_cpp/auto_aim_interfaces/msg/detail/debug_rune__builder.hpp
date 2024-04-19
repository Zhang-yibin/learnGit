// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/DebugRune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/debug_rune__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_DebugRune_c_function
{
public:
  explicit Init_DebugRune_c_function(::auto_aim_interfaces::msg::DebugRune & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::DebugRune c_function(::auto_aim_interfaces::msg::DebugRune::_c_function_type arg)
  {
    msg_.c_function = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugRune msg_;
};

class Init_DebugRune_filter_speed
{
public:
  explicit Init_DebugRune_filter_speed(::auto_aim_interfaces::msg::DebugRune & msg)
  : msg_(msg)
  {}
  Init_DebugRune_c_function filter_speed(::auto_aim_interfaces::msg::DebugRune::_filter_speed_type arg)
  {
    msg_.filter_speed = std::move(arg);
    return Init_DebugRune_c_function(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugRune msg_;
};

class Init_DebugRune_spin_speed
{
public:
  explicit Init_DebugRune_spin_speed(::auto_aim_interfaces::msg::DebugRune & msg)
  : msg_(msg)
  {}
  Init_DebugRune_filter_speed spin_speed(::auto_aim_interfaces::msg::DebugRune::_spin_speed_type arg)
  {
    msg_.spin_speed = std::move(arg);
    return Init_DebugRune_filter_speed(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugRune msg_;
};

class Init_DebugRune_header
{
public:
  Init_DebugRune_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugRune_spin_speed header(::auto_aim_interfaces::msg::DebugRune::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DebugRune_spin_speed(msg_);
  }

private:
  ::auto_aim_interfaces::msg::DebugRune msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::DebugRune>()
{
  return auto_aim_interfaces::msg::builder::Init_DebugRune_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__BUILDER_HPP_
