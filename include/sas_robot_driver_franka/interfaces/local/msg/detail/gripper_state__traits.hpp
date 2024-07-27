// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/msg/gripper_state.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__TRAITS_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sas_robot_driver_franka_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperState & msg,
  std::ostream & out)
{
  out << "{";
  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: max_width
  {
    out << "max_width: ";
    rosidl_generator_traits::value_to_yaml(msg.max_width, out);
    out << ", ";
  }

  // member: is_grasped
  {
    out << "is_grasped: ";
    rosidl_generator_traits::value_to_yaml(msg.is_grasped, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: duration_ms
  {
    out << "duration_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.duration_ms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: max_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_width: ";
    rosidl_generator_traits::value_to_yaml(msg.max_width, out);
    out << "\n";
  }

  // member: is_grasped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_grasped: ";
    rosidl_generator_traits::value_to_yaml(msg.is_grasped, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: duration_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.duration_ms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperState & msg, bool use_flow_style = false)
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

}  // namespace sas_robot_driver_franka_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sas_robot_driver_franka_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sas_robot_driver_franka_interfaces::msg::GripperState & msg,
  std::ostream & out, size_t indentation = 0)
{
  sas_robot_driver_franka_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sas_robot_driver_franka_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sas_robot_driver_franka_interfaces::msg::GripperState & msg)
{
  return sas_robot_driver_franka_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sas_robot_driver_franka_interfaces::msg::GripperState>()
{
  return "sas_robot_driver_franka_interfaces::msg::GripperState";
}

template<>
inline const char * name<sas_robot_driver_franka_interfaces::msg::GripperState>()
{
  return "sas_robot_driver_franka_interfaces/msg/GripperState";
}

template<>
struct has_fixed_size<sas_robot_driver_franka_interfaces::msg::GripperState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sas_robot_driver_franka_interfaces::msg::GripperState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sas_robot_driver_franka_interfaces::msg::GripperState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__TRAITS_HPP_
