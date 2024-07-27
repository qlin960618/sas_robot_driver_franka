// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice
#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_sas_robot_driver_franka_interfaces__msg__GripperState(
  const sas_robot_driver_franka_interfaces__msg__GripperState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_deserialize_sas_robot_driver_franka_interfaces__msg__GripperState(
  eprosima::fastcdr::Cdr &,
  sas_robot_driver_franka_interfaces__msg__GripperState * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_sas_robot_driver_franka_interfaces__msg__GripperState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_sas_robot_driver_franka_interfaces__msg__GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_key_sas_robot_driver_franka_interfaces__msg__GripperState(
  const sas_robot_driver_franka_interfaces__msg__GripperState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_key_sas_robot_driver_franka_interfaces__msg__GripperState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_key_sas_robot_driver_franka_interfaces__msg__GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sas_robot_driver_franka_interfaces, msg, GripperState)();

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
