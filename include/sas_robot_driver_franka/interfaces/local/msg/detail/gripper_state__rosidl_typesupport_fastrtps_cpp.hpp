// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace sas_robot_driver_franka_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
cdr_serialize(
  const sas_robot_driver_franka_interfaces::msg::GripperState & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sas_robot_driver_franka_interfaces::msg::GripperState & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
get_serialized_size(
  const sas_robot_driver_franka_interfaces::msg::GripperState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
max_serialized_size_GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
cdr_serialize_key(
  const sas_robot_driver_franka_interfaces::msg::GripperState & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
get_serialized_size_key(
  const sas_robot_driver_franka_interfaces::msg::GripperState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
max_serialized_size_key_GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace sas_robot_driver_franka_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sas_robot_driver_franka_interfaces, msg, GripperState)();

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
