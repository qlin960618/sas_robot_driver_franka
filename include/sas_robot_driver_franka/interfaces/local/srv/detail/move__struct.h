// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sas_robot_driver_franka_interfaces:srv/Move.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/srv/move.h"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__STRUCT_H_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Move in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__srv__Move_Request
{
  float width;
  float speed;
} sas_robot_driver_franka_interfaces__srv__Move_Request;

// Struct for a sequence of sas_robot_driver_franka_interfaces__srv__Move_Request.
typedef struct sas_robot_driver_franka_interfaces__srv__Move_Request__Sequence
{
  sas_robot_driver_franka_interfaces__srv__Move_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__srv__Move_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Move in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__srv__Move_Response
{
  bool success;
} sas_robot_driver_franka_interfaces__srv__Move_Response;

// Struct for a sequence of sas_robot_driver_franka_interfaces__srv__Move_Response.
typedef struct sas_robot_driver_franka_interfaces__srv__Move_Response__Sequence
{
  sas_robot_driver_franka_interfaces__srv__Move_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__srv__Move_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  sas_robot_driver_franka_interfaces__srv__Move_Event__request__MAX_SIZE = 1
};
// response
enum
{
  sas_robot_driver_franka_interfaces__srv__Move_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Move in the package sas_robot_driver_franka_interfaces.
typedef struct sas_robot_driver_franka_interfaces__srv__Move_Event
{
  service_msgs__msg__ServiceEventInfo info;
  sas_robot_driver_franka_interfaces__srv__Move_Request__Sequence request;
  sas_robot_driver_franka_interfaces__srv__Move_Response__Sequence response;
} sas_robot_driver_franka_interfaces__srv__Move_Event;

// Struct for a sequence of sas_robot_driver_franka_interfaces__srv__Move_Event.
typedef struct sas_robot_driver_franka_interfaces__srv__Move_Event__Sequence
{
  sas_robot_driver_franka_interfaces__srv__Move_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sas_robot_driver_franka_interfaces__srv__Move_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__STRUCT_H_
