// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__functions.h"
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace sas_robot_driver_franka_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GripperState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sas_robot_driver_franka_interfaces::msg::GripperState(_init);
}

void GripperState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sas_robot_driver_franka_interfaces::msg::GripperState *>(message_memory);
  typed_message->~GripperState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GripperState_message_member_array[5] = {
  {
    "width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces::msg::GripperState, width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces::msg::GripperState, max_width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_grasped",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces::msg::GripperState, is_grasped),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces::msg::GripperState, temperature),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "duration_ms",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces::msg::GripperState, duration_ms),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GripperState_message_members = {
  "sas_robot_driver_franka_interfaces::msg",  // message namespace
  "GripperState",  // message name
  5,  // number of fields
  sizeof(sas_robot_driver_franka_interfaces::msg::GripperState),
  false,  // has_any_key_member_
  GripperState_message_member_array,  // message members
  GripperState_init_function,  // function to initialize message memory (memory has to be allocated)
  GripperState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GripperState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GripperState_message_members,
  get_message_typesupport_handle_function,
  &sas_robot_driver_franka_interfaces__msg__GripperState__get_type_hash,
  &sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description,
  &sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sas_robot_driver_franka_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sas_robot_driver_franka_interfaces::msg::GripperState>()
{
  return &::sas_robot_driver_franka_interfaces::msg::rosidl_typesupport_introspection_cpp::GripperState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sas_robot_driver_franka_interfaces, msg, GripperState)() {
  return &::sas_robot_driver_franka_interfaces::msg::rosidl_typesupport_introspection_cpp::GripperState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
