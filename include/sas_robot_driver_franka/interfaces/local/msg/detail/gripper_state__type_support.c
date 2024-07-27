// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__rosidl_typesupport_introspection_c.h"
#include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__functions.h"
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sas_robot_driver_franka_interfaces__msg__GripperState__init(message_memory);
}

void sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_fini_function(void * message_memory)
{
  sas_robot_driver_franka_interfaces__msg__GripperState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_member_array[5] = {
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__msg__GripperState, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__msg__GripperState, max_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_grasped",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__msg__GripperState, is_grasped),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__msg__GripperState, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__msg__GripperState, duration_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_members = {
  "sas_robot_driver_franka_interfaces__msg",  // message namespace
  "GripperState",  // message name
  5,  // number of fields
  sizeof(sas_robot_driver_franka_interfaces__msg__GripperState),
  false,  // has_any_key_member_
  sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_member_array,  // message members
  sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_init_function,  // function to initialize message memory (memory has to be allocated)
  sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle = {
  0,
  &sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_members,
  get_message_typesupport_handle_function,
  &sas_robot_driver_franka_interfaces__msg__GripperState__get_type_hash,
  &sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description,
  &sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, msg, GripperState)() {
  if (!sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle.typesupport_identifier) {
    sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sas_robot_driver_franka_interfaces__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
