// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sas_robot_driver_franka_interfaces:srv/Grasp.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/srv/grasp.h"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__STRUCT_H_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Grasp in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__srv__Grasp_Request
{
  float width;
  float speed;
  float force;
  float epsilon_inner;
  float epsilon_outer;
} sas_robot_driver_franka_interfaces__srv__Grasp_Request;

// Struct for a sequence of sas_robot_driver_franka_interfaces__srv__Grasp_Request.
typedef struct sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Grasp in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__srv__Grasp_Response
{
  bool success;
} sas_robot_driver_franka_interfaces__srv__Grasp_Response;

// Struct for a sequence of sas_robot_driver_franka_interfaces__srv__Grasp_Response.
typedef struct sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__request__MAX_SIZE = 1
};
// response
enum
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Grasp in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__srv__Grasp_Event
{
  service_msgs__msg__ServiceEventInfo info;
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence request;
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence response;
} sas_robot_driver_franka_interfaces__srv__Grasp_Event;

// Struct for a sequence of sas_robot_driver_franka_interfaces__srv__Grasp_Event.
typedef struct sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence
{
  sas_robot_driver_franka_interfaces__srv__Grasp_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__STRUCT_H_
