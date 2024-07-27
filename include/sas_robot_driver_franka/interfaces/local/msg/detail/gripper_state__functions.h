// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/msg/gripper_state.h"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__FUNCTIONS_H_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "sas_robot_driver_franka_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__struct.h"

/// Initialize msg/GripperState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sas_robot_driver_franka_interfaces__msg__GripperState
 * )) before or use
 * sas_robot_driver_franka_interfaces__msg__GripperState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
bool
sas_robot_driver_franka_interfaces__msg__GripperState__init(sas_robot_driver_franka_interfaces__msg__GripperState * msg);

/// Finalize msg/GripperState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
void
sas_robot_driver_franka_interfaces__msg__GripperState__fini(sas_robot_driver_franka_interfaces__msg__GripperState * msg);

/// Create msg/GripperState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sas_robot_driver_franka_interfaces__msg__GripperState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
sas_robot_driver_franka_interfaces__msg__GripperState *
sas_robot_driver_franka_interfaces__msg__GripperState__create(void);

/// Destroy msg/GripperState message.
/**
 * It calls
 * sas_robot_driver_franka_interfaces__msg__GripperState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
void
sas_robot_driver_franka_interfaces__msg__GripperState__destroy(sas_robot_driver_franka_interfaces__msg__GripperState * msg);

/// Check for msg/GripperState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
bool
sas_robot_driver_franka_interfaces__msg__GripperState__are_equal(const sas_robot_driver_franka_interfaces__msg__GripperState * lhs, const sas_robot_driver_franka_interfaces__msg__GripperState * rhs);

/// Copy a msg/GripperState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
bool
sas_robot_driver_franka_interfaces__msg__GripperState__copy(
  const sas_robot_driver_franka_interfaces__msg__GripperState * input,
  sas_robot_driver_franka_interfaces__msg__GripperState * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_type_hash_t *
sas_robot_driver_franka_interfaces__msg__GripperState__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_runtime_c__type_description__TypeSource *
sas_robot_driver_franka_interfaces__msg__GripperState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/GripperState messages.
/**
 * It allocates the memory for the number of elements and calls
 * sas_robot_driver_franka_interfaces__msg__GripperState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
bool
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__init(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array, size_t size);

/// Finalize array of msg/GripperState messages.
/**
 * It calls
 * sas_robot_driver_franka_interfaces__msg__GripperState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
void
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__fini(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array);

/// Create array of msg/GripperState messages.
/**
 * It allocates the memory for the array and calls
 * sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence *
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__create(size_t size);

/// Destroy array of msg/GripperState messages.
/**
 * It calls
 * sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
void
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__destroy(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array);

/// Check for msg/GripperState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
bool
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__are_equal(const sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * lhs, const sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * rhs);

/// Copy an array of msg/GripperState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
bool
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__copy(
  const sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * input,
  sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__FUNCTIONS_H_
