// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/rune__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'r_center'
// Member 'target_center'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Rune & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: rune_mode
  {
    out << "rune_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.rune_mode, out);
    out << ", ";
  }

  // member: r_center
  {
    out << "r_center: ";
    to_flow_style_yaml(msg.r_center, out);
    out << ", ";
  }

  // member: target_center
  {
    out << "target_center: ";
    to_flow_style_yaml(msg.target_center, out);
    out << ", ";
  }

  // member: real_angle
  {
    out << "real_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.real_angle, out);
    out << ", ";
  }

  // member: radian
  {
    out << "radian: ";
    rosidl_generator_traits::value_to_yaml(msg.radian, out);
    out << ", ";
  }

  // member: rows
  {
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << ", ";
  }

  // member: cols
  {
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Rune & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: rune_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rune_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.rune_mode, out);
    out << "\n";
  }

  // member: r_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_center:\n";
    to_block_style_yaml(msg.r_center, out, indentation + 2);
  }

  // member: target_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_center:\n";
    to_block_style_yaml(msg.target_center, out, indentation + 2);
  }

  // member: real_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.real_angle, out);
    out << "\n";
  }

  // member: radian
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radian: ";
    rosidl_generator_traits::value_to_yaml(msg.radian, out);
    out << "\n";
  }

  // member: rows
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << "\n";
  }

  // member: cols
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Rune & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::Rune & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::Rune & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::Rune>()
{
  return "auto_aim_interfaces::msg::Rune";
}

template<>
inline const char * name<auto_aim_interfaces::msg::Rune>()
{
  return "auto_aim_interfaces/msg/Rune";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::Rune>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::Rune>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<auto_aim_interfaces::msg::Rune>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RUNE__TRAITS_HPP_
