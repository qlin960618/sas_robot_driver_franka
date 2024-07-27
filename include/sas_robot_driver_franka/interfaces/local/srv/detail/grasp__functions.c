// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sas_robot_driver_franka_interfaces:srv/Grasp.idl
// generated code does not contain a copyright notice
#include "sas_robot_driver_franka_interfaces/srv/detail/grasp__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Request__init(sas_robot_driver_franka_interfaces__srv__Grasp_Request * msg)
{
  if (!msg) {
    return false;
  }
  // width
  // speed
  // force
  // epsilon_inner
  // epsilon_outer
  return true;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Request__fini(sas_robot_driver_franka_interfaces__srv__Grasp_Request * msg)
{
  if (!msg) {
    return;
  }
  // width
  // speed
  // force
  // epsilon_inner
  // epsilon_outer
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Request__are_equal(const sas_robot_driver_franka_interfaces__srv__Grasp_Request * lhs, const sas_robot_driver_franka_interfaces__srv__Grasp_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  // epsilon_inner
  if (lhs->epsilon_inner != rhs->epsilon_inner) {
    return false;
  }
  // epsilon_outer
  if (lhs->epsilon_outer != rhs->epsilon_outer) {
    return false;
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Request__copy(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request * input,
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // width
  output->width = input->width;
  // speed
  output->speed = input->speed;
  // force
  output->force = input->force;
  // epsilon_inner
  output->epsilon_inner = input->epsilon_inner;
  // epsilon_outer
  output->epsilon_outer = input->epsilon_outer;
  return true;
}

sas_robot_driver_franka_interfaces__srv__Grasp_Request *
sas_robot_driver_franka_interfaces__srv__Grasp_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * msg = (sas_robot_driver_franka_interfaces__srv__Grasp_Request *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Request));
  bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Request__destroy(sas_robot_driver_franka_interfaces__srv__Grasp_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__init(sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Request * data = NULL;

  if (size) {
    data = (sas_robot_driver_franka_interfaces__srv__Grasp_Request *)allocator.zero_allocate(size, sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sas_robot_driver_franka_interfaces__srv__Grasp_Request__fini(&data[i - 1]);
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
sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__fini(sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * array)
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
      sas_robot_driver_franka_interfaces__srv__Grasp_Request__fini(&array->data[i]);
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

sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence *
sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * array = (sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__destroy(sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__are_equal(const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * lhs, const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__copy(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * input,
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sas_robot_driver_franka_interfaces__srv__Grasp_Request * data =
      (sas_robot_driver_franka_interfaces__srv__Grasp_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sas_robot_driver_franka_interfaces__srv__Grasp_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
sas_robot_driver_franka_interfaces__srv__Grasp_Response__init(sas_robot_driver_franka_interfaces__srv__Grasp_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Response__fini(sas_robot_driver_franka_interfaces__srv__Grasp_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Response__are_equal(const sas_robot_driver_franka_interfaces__srv__Grasp_Response * lhs, const sas_robot_driver_franka_interfaces__srv__Grasp_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Response__copy(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response * input,
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

sas_robot_driver_franka_interfaces__srv__Grasp_Response *
sas_robot_driver_franka_interfaces__srv__Grasp_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * msg = (sas_robot_driver_franka_interfaces__srv__Grasp_Response *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Response));
  bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Response__destroy(sas_robot_driver_franka_interfaces__srv__Grasp_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__init(sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Response * data = NULL;

  if (size) {
    data = (sas_robot_driver_franka_interfaces__srv__Grasp_Response *)allocator.zero_allocate(size, sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sas_robot_driver_franka_interfaces__srv__Grasp_Response__fini(&data[i - 1]);
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
sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__fini(sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * array)
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
      sas_robot_driver_franka_interfaces__srv__Grasp_Response__fini(&array->data[i]);
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

sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence *
sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * array = (sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__destroy(sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__are_equal(const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * lhs, const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__copy(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * input,
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sas_robot_driver_franka_interfaces__srv__Grasp_Response * data =
      (sas_robot_driver_franka_interfaces__srv__Grasp_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sas_robot_driver_franka_interfaces__srv__Grasp_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "sas_robot_driver_franka_interfaces/srv/detail/grasp__functions.h"

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Event__init(sas_robot_driver_franka_interfaces__srv__Grasp_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(msg);
    return false;
  }
  // request
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__init(&msg->request, 0)) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(msg);
    return false;
  }
  // response
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__init(&msg->response, 0)) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(msg);
    return false;
  }
  return true;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(sas_robot_driver_franka_interfaces__srv__Grasp_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__fini(&msg->request);
  // response
  sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__fini(&msg->response);
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Event__are_equal(const sas_robot_driver_franka_interfaces__srv__Grasp_Event * lhs, const sas_robot_driver_franka_interfaces__srv__Grasp_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Event__copy(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Event * input,
  sas_robot_driver_franka_interfaces__srv__Grasp_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!sas_robot_driver_franka_interfaces__srv__Grasp_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

sas_robot_driver_franka_interfaces__srv__Grasp_Event *
sas_robot_driver_franka_interfaces__srv__Grasp_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Event * msg = (sas_robot_driver_franka_interfaces__srv__Grasp_Event *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Event));
  bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Event__destroy(sas_robot_driver_franka_interfaces__srv__Grasp_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__init(sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Event * data = NULL;

  if (size) {
    data = (sas_robot_driver_franka_interfaces__srv__Grasp_Event *)allocator.zero_allocate(size, sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(&data[i - 1]);
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
sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__fini(sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * array)
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
      sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(&array->data[i]);
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

sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence *
sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * array = (sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence *)allocator.allocate(sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__destroy(sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__are_equal(const sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * lhs, const sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__srv__Grasp_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence__copy(
  const sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * input,
  sas_robot_driver_franka_interfaces__srv__Grasp_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sas_robot_driver_franka_interfaces__srv__Grasp_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sas_robot_driver_franka_interfaces__srv__Grasp_Event * data =
      (sas_robot_driver_franka_interfaces__srv__Grasp_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sas_robot_driver_franka_interfaces__srv__Grasp_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sas_robot_driver_franka_interfaces__srv__Grasp_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sas_robot_driver_franka_interfaces__srv__Grasp_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
