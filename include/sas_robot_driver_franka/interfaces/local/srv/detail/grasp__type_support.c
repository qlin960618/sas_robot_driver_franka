// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sas_robot_driver_franka_interfaces:srv/Grasp.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sas_robot_driver_franka_interfaces/srv/detail/grasp__rosidl_typesupport_introspection_c.h"
#include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sas_robot_driver_franka_interfaces/srv/detail/grasp__functions.h"
#include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__init(message_memory);
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_fini_function(void * message_memory)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_member_array[5] = {
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Request, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Request, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Request, force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "epsilon_inner",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Request, epsilon_inner),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "epsilon_outer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Request, epsilon_outer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_members = {
  "sas_robot_driver_franka_interfaces__srv",  // message namespace
  "Grasp_Request",  // message name
  5,  // number of fields
  sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Request),
  false,  // has_any_key_member_
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_member_array,  // message members
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_type_support_handle = {
  0,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_members,
  get_message_typesupport_handle_function,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Request__get_type_hash,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Request__get_type_description,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Request)() {
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_type_support_handle.typesupport_identifier) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__rosidl_typesupport_introspection_c.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__functions.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__init(message_memory);
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_fini_function(void * message_memory)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_members = {
  "sas_robot_driver_franka_interfaces__srv",  // message namespace
  "Grasp_Response",  // message name
  1,  // number of fields
  sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Response),
  false,  // has_any_key_member_
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_member_array,  // message members
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle = {
  0,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_members,
  get_message_typesupport_handle_function,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Response__get_type_hash,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Response__get_type_description,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Response)() {
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle.typesupport_identifier) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__rosidl_typesupport_introspection_c.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__functions.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "sas_robot_driver_franka_interfaces/srv/grasp.h"
// Member `request`
// Member `response`
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__init(message_memory);
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_fini_function(void * message_memory)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(message_memory);
}

size_t sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__size_function__Grasp_Event__request(
  const void * untyped_member)
{
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * member =
    (const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_const_function__Grasp_Event__request(
  const void * untyped_member, size_t index)
{
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * member =
    (const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_function__Grasp_Event__request(
  void * untyped_member, size_t index)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * member =
    (sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__fetch_function__Grasp_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request * item =
    ((const sas_robot_driver_franka_interfaces__srv__Grasp_Request *)
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_const_function__Grasp_Event__request(untyped_member, index));
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * value =
    (sas_robot_driver_franka_interfaces__srv__Grasp_Request *)(untyped_value);
  *value = *item;
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__assign_function__Grasp_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * item =
    ((sas_robot_driver_franka_interfaces__srv__Grasp_Request *)
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_function__Grasp_Event__request(untyped_member, index));
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request * value =
    (const sas_robot_driver_franka_interfaces__srv__Grasp_Request *)(untyped_value);
  *item = *value;
}

bool sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__resize_function__Grasp_Event__request(
  void * untyped_member, size_t size)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * member =
    (sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence *)(untyped_member);
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__fini(member);
  return sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__init(member, size);
}

size_t sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__size_function__Grasp_Event__response(
  const void * untyped_member)
{
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * member =
    (const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_const_function__Grasp_Event__response(
  const void * untyped_member, size_t index)
{
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * member =
    (const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_function__Grasp_Event__response(
  void * untyped_member, size_t index)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * member =
    (sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__fetch_function__Grasp_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response * item =
    ((const sas_robot_driver_franka_interfaces__srv__Grasp_Response *)
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_const_function__Grasp_Event__response(untyped_member, index));
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * value =
    (sas_robot_driver_franka_interfaces__srv__Grasp_Response *)(untyped_value);
  *value = *item;
}

void sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__assign_function__Grasp_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * item =
    ((sas_robot_driver_franka_interfaces__srv__Grasp_Response *)
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_function__Grasp_Event__response(untyped_member, index));
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response * value =
    (const sas_robot_driver_franka_interfaces__srv__Grasp_Response *)(untyped_value);
  *item = *value;
}

bool sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__resize_function__Grasp_Event__response(
  void * untyped_member, size_t size)
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * member =
    (sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence *)(untyped_member);
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__fini(member);
  return sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Event, request),  // bytes offset in struct
    NULL,  // default value
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__size_function__Grasp_Event__request,  // size() function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_const_function__Grasp_Event__request,  // get_const(index) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_function__Grasp_Event__request,  // get(index) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__fetch_function__Grasp_Event__request,  // fetch(index, &value) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__assign_function__Grasp_Event__request,  // assign(index, value) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__resize_function__Grasp_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(sas_robot_driver_franka_interfaces__srv__Grasp_Event, response),  // bytes offset in struct
    NULL,  // default value
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__size_function__Grasp_Event__response,  // size() function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_const_function__Grasp_Event__response,  // get_const(index) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__get_function__Grasp_Event__response,  // get(index) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__fetch_function__Grasp_Event__response,  // fetch(index, &value) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__assign_function__Grasp_Event__response,  // assign(index, value) function pointer
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__resize_function__Grasp_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_members = {
  "sas_robot_driver_franka_interfaces__srv",  // message namespace
  "Grasp_Event",  // message name
  3,  // number of fields
  sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Event),
  false,  // has_any_key_member_
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_member_array,  // message members
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_type_support_handle = {
  0,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_members,
  get_message_typesupport_handle_function,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Event__get_type_hash,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Event__get_type_description,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Event)() {
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Request)();
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Response)();
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_type_support_handle.typesupport_identifier) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_members = {
  "sas_robot_driver_franka_interfaces__srv",  // service namespace
  "Grasp",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_Request_message_type_support_handle,
  NULL,  // response message
  // sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle
  NULL  // event_message
  // sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle
};


static rosidl_service_type_support_t sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_type_support_handle = {
  0,
  &sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_members,
  get_service_typesupport_handle_function,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Request__rosidl_typesupport_introspection_c__Grasp_Request_message_type_support_handle,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Response__rosidl_typesupport_introspection_c__Grasp_Response_message_type_support_handle,
  &sas_robot_driver_franka_interfaces__srv__Grasp_Event__rosidl_typesupport_introspection_c__Grasp_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    sas_robot_driver_franka_interfaces,
    srv,
    Grasp
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    sas_robot_driver_franka_interfaces,
    srv,
    Grasp
  ),
  &sas_robot_driver_franka_interfaces__srv__Grasp__get_type_hash,
  &sas_robot_driver_franka_interfaces__srv__Grasp__get_type_description,
  &sas_robot_driver_franka_interfaces__srv__Grasp__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sas_robot_driver_franka_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp)(void) {
  if (!sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_type_support_handle.typesupport_identifier) {
    sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sas_robot_driver_franka_interfaces, srv, Grasp_Event)()->data;
  }

  return &sas_robot_driver_franka_interfaces__srv__detail__grasp__rosidl_typesupport_introspection_c__Grasp_service_type_support_handle;
}
