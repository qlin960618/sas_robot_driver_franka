// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice
#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sas_robot_driver_franka_interfaces__msg__GripperState__init(sas_robot_driver_franka_interfaces__msg__GripperState * msg)
{
  if (!msg) {
    return false;
  }
  // width
  // max_width
  // is_grasped
  // temperature
  // duration_ms
  return true;
}

void
sas_robot_driver_franka_interfaces__msg__GripperState__fini(sas_robot_driver_franka_interfaces__msg__GripperState * msg)
{
  if (!msg) {
    return;
  }
  // width
  // max_width
  // is_grasped
  // temperature
  // duration_ms
}

bool
sas_robot_driver_franka_interfaces__msg__GripperState__are_equal(const sas_robot_driver_franka_interfaces__msg__GripperState * lhs, const sas_robot_driver_franka_interfaces__msg__GripperState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // max_width
  if (lhs->max_width != rhs->max_width) {
    return false;
  }
  // is_grasped
  if (lhs->is_grasped != rhs->is_grasped) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // duration_ms
  if (lhs->duration_ms != rhs->duration_ms) {
    return false;
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__msg__GripperState__copy(
  const sas_robot_driver_franka_interfaces__msg__GripperState * input,
  sas_robot_driver_franka_interfaces__msg__GripperState * output)
{
  if (!input || !output) {
    return false;
  }
  // width
  output->width = input->width;
  // max_width
  output->max_width = input->max_width;
  // is_grasped
  output->is_grasped = input->is_grasped;
  // temperature
  output->temperature = input->temperature;
  // duration_ms
  output->duration_ms = input->duration_ms;
  return true;
}

sas_robot_driver_franka_interfaces__msg__GripperState *
sas_robot_driver_franka_interfaces__msg__GripperState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__msg__GripperState * msg = (sas_robot_driver_franka_interfaces__msg__GripperState *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__msg__GripperState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sas_robot_driver_franka_interfaces__msg__GripperState));
  bool success = sas_robot_driver_franka_interfaces__msg__GripperState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sas_robot_driver_franka_interfaces__msg__GripperState__destroy(sas_robot_driver_franka_interfaces__msg__GripperState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sas_robot_driver_franka_interfaces__msg__GripperState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__init(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__msg__GripperState * data = NULL;

  if (size) {
    data = (sas_robot_driver_franka_interfaces__msg__GripperState *)allocator.zero_allocate(size, sizeof(sas_robot_driver_franka_interfaces__msg__GripperState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sas_robot_driver_franka_interfaces__msg__GripperState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sas_robot_driver_franka_interfaces__msg__GripperState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__fini(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sas_robot_driver_franka_interfaces__msg__GripperState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sas_robot_driver_franka_interfaces__msg__GripperState__Sequence *
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array = (sas_robot_driver_franka_interfaces__msg__GripperState__Sequence *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__destroy(sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__are_equal(const sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * lhs, const sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__msg__GripperState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__msg__GripperState__Sequence__copy(
  const sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * input,
  sas_robot_driver_franka_interfaces__msg__GripperState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sas_robot_driver_franka_interfaces__msg__GripperState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sas_robot_driver_franka_interfaces__msg__GripperState * data =
      (sas_robot_driver_franka_interfaces__msg__GripperState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sas_robot_driver_franka_interfaces__msg__GripperState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sas_robot_driver_franka_interfaces__msg__GripperState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__msg__GripperState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
