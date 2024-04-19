// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/rune__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Rune_cols
{
public:
  explicit Init_Rune_cols(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Rune cols(::auto_aim_interfaces::msg::Rune::_cols_type arg)
  {
    msg_.cols = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_rows
{
public:
  explicit Init_Rune_rows(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_cols rows(::auto_aim_interfaces::msg::Rune::_rows_type arg)
  {
    msg_.rows = std::move(arg);
    return Init_Rune_cols(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_radian
{
public:
  explicit Init_Rune_radian(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_rows radian(::auto_aim_interfaces::msg::Rune::_radian_type arg)
  {
    msg_.radian = std::move(arg);
    return Init_Rune_rows(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_real_angle
{
public:
  explicit Init_Rune_real_angle(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_radian real_angle(::auto_aim_interfaces::msg::Rune::_real_angle_type arg)
  {
    msg_.real_angle = std::move(arg);
    return Init_Rune_radian(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_target_center
{
public:
  explicit Init_Rune_target_center(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_real_angle target_center(::auto_aim_interfaces::msg::Rune::_target_center_type arg)
  {
    msg_.target_center = std::move(arg);
    return Init_Rune_real_angle(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_r_center
{
public:
  explicit Init_Rune_r_center(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_target_center r_center(::auto_aim_interfaces::msg::Rune::_r_center_type arg)
  {
    msg_.r_center = std::move(arg);
    return Init_Rune_target_center(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_rune_mode
{
public:
  explicit Init_Rune_rune_mode(::auto_aim_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_r_center rune_mode(::auto_aim_interfaces::msg::Rune::_rune_mode_type arg)
  {
    msg_.rune_mode = std::move(arg);
    return Init_Rune_r_center(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

class Init_Rune_header
{
public:
  Init_Rune_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Rune_rune_mode header(::auto_aim_interfaces::msg::Rune::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Rune_rune_mode(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Rune msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Rune>()
{
  return auto_aim_interfaces::msg::builder::Init_Rune_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__BUILDER_HPP_
