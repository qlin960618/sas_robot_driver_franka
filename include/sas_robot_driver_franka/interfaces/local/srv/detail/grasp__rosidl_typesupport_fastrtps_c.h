// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from sas_robot_driver_franka_interfaces:srv/Grasp.idl
// generated code does not contain a copyright notice
#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_deserialize_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  eprosima::fastcdr::Cdr &,
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_key_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_key_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_key_sas_robot_driver_franka_interfaces__srv__Grasp_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sas_robot_driver_franka_interfaces, srv, Grasp_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_deserialize_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  eprosima::fastcdr::Cdr &,
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_key_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_key_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_key_sas_robot_driver_franka_interfaces__srv__Grasp_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sas_robot_driver_franka_interfaces, srv, Grasp_Response)();

#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_deserialize_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  eprosima::fastcdr::Cdr &,
  sas_robot_driver_franka_interfaces__srv__Grasp_Event * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
bool cdr_serialize_key_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t get_serialized_size_key_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
size_t max_serialized_size_key_sas_robot_driver_franka_interfaces__srv__Grasp_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sas_robot_driver_franka_interfaces, srv, Grasp_Event)();

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "sas_robot_driver_franka_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sas_robot_driver_franka_interfaces, srv, Grasp)();

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
