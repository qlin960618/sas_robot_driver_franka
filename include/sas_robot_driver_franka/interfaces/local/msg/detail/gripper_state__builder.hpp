// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/msg/gripper_state.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__BUILDER_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sas_robot_driver_franka_interfaces
{

namespace msg
{

namespace builder
{

class Init_GripperState_duration_ms
{
public:
  explicit Init_GripperState_duration_ms(::sas_robot_driver_franka_interfaces::msg::GripperState & msg)
  : msg_(msg)
  {}
  ::sas_robot_driver_franka_interfaces::msg::GripperState duration_ms(::sas_robot_driver_franka_interfaces::msg::GripperState::_duration_ms_type arg)
  {
    msg_.duration_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::msg::GripperState msg_;
};

class Init_GripperState_temperature
{
public:
  explicit Init_GripperState_temperature(::sas_robot_driver_franka_interfaces::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_duration_ms temperature(::sas_robot_driver_franka_interfaces::msg::GripperState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_GripperState_duration_ms(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::msg::GripperState msg_;
};

class Init_GripperState_is_grasped
{
public:
  explicit Init_GripperState_is_grasped(::sas_robot_driver_franka_interfaces::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_temperature is_grasped(::sas_robot_driver_franka_interfaces::msg::GripperState::_is_grasped_type arg)
  {
    msg_.is_grasped = std::move(arg);
    return Init_GripperState_temperature(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::msg::GripperState msg_;
};

class Init_GripperState_max_width
{
public:
  explicit Init_GripperState_max_width(::sas_robot_driver_franka_interfaces::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_is_grasped max_width(::sas_robot_driver_franka_interfaces::msg::GripperState::_max_width_type arg)
  {
    msg_.max_width = std::move(arg);
    return Init_GripperState_is_grasped(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::msg::GripperState msg_;
};

class Init_GripperState_width
{
public:
  Init_GripperState_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperState_max_width width(::sas_robot_driver_franka_interfaces::msg::GripperState::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_GripperState_max_width(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::msg::GripperState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::msg::GripperState>()
{
  return sas_robot_driver_franka_interfaces::msg::builder::Init_GripperState_width();
}

}  // namespace sas_robot_driver_franka_interfaces

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__BUILDER_HPP_
