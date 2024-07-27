// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/msg/gripper_state.h"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__STRUCT_H_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/GripperState in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__msg__GripperState
{
  float width;
  float max_width;
  bool is_grasped;
  uint16_t temperature;
  uint64_t duration_ms;
} sas_robot_driver_franka_interfaces__msg__GripperState;

// Struct for a sequence of sas_robot_driver_franka_interfaces__msg__GripperState.
typedef struct sas_robot_driver_franka_interfaces__msg__GripperState__Sequence
{
  sas_robot_driver_franka_interfaces__msg__GripperState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__msg__GripperState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__STRUCT_H_
