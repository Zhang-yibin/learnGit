// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/DebugRune.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/debug_rune__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DebugRune & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: spin_speed
  {
    out << "spin_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.spin_speed, out);
    out << ", ";
  }

  // member: filter_speed
  {
    out << "filter_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.filter_speed, out);
    out << ", ";
  }

  // member: c_function
  {
    out << "c_function: ";
    rosidl_generator_traits::value_to_yaml(msg.c_function, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DebugRune & msg,
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

  // member: spin_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spin_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.spin_speed, out);
    out << "\n";
  }

  // member: filter_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filter_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.filter_speed, out);
    out << "\n";
  }

  // member: c_function
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c_function: ";
    rosidl_generator_traits::value_to_yaml(msg.c_function, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DebugRune & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::DebugRune & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::DebugRune & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::DebugRune>()
{
  return "auto_aim_interfaces::msg::DebugRune";
}

template<>
inline const char * name<auto_aim_interfaces::msg::DebugRune>()
{
  return "auto_aim_interfaces/msg/DebugRune";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::DebugRune>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::DebugRune>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<auto_aim_interfaces::msg::DebugRune>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_RUNE__TRAITS_HPP_
