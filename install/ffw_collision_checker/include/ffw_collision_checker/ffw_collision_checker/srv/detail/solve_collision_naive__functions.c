// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ffw_collision_checker:srv/SolveCollisionNaive.idl
// generated code does not contain a copyright notice
#include "ffw_collision_checker/srv/detail/solve_collision_naive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"

bool
ffw_collision_checker__srv__SolveCollisionNaive_Request__init(ffw_collision_checker__srv__SolveCollisionNaive_Request * msg)
{
  if (!msg) {
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->joint_names, 0)) {
    ffw_collision_checker__srv__SolveCollisionNaive_Request__fini(msg);
    return false;
  }
  return true;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Request__fini(ffw_collision_checker__srv__SolveCollisionNaive_Request * msg)
{
  if (!msg) {
    return;
  }
  // joint_names
  rosidl_runtime_c__String__Sequence__fini(&msg->joint_names);
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Request__are_equal(const ffw_collision_checker__srv__SolveCollisionNaive_Request * lhs, const ffw_collision_checker__srv__SolveCollisionNaive_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->joint_names), &(rhs->joint_names)))
  {
    return false;
  }
  return true;
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Request__copy(
  const ffw_collision_checker__srv__SolveCollisionNaive_Request * input,
  ffw_collision_checker__srv__SolveCollisionNaive_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->joint_names), &(output->joint_names)))
  {
    return false;
  }
  return true;
}

ffw_collision_checker__srv__SolveCollisionNaive_Request *
ffw_collision_checker__srv__SolveCollisionNaive_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Request * msg = (ffw_collision_checker__srv__SolveCollisionNaive_Request *)allocator.allocate(sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Request));
  bool success = ffw_collision_checker__srv__SolveCollisionNaive_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Request__destroy(ffw_collision_checker__srv__SolveCollisionNaive_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ffw_collision_checker__srv__SolveCollisionNaive_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__init(ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Request * data = NULL;

  if (size) {
    data = (ffw_collision_checker__srv__SolveCollisionNaive_Request *)allocator.zero_allocate(size, sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ffw_collision_checker__srv__SolveCollisionNaive_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ffw_collision_checker__srv__SolveCollisionNaive_Request__fini(&data[i - 1]);
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
ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__fini(ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * array)
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
      ffw_collision_checker__srv__SolveCollisionNaive_Request__fini(&array->data[i]);
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

ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence *
ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * array = (ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence *)allocator.allocate(sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__destroy(ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__are_equal(const ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * lhs, const ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ffw_collision_checker__srv__SolveCollisionNaive_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__copy(
  const ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * input,
  ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ffw_collision_checker__srv__SolveCollisionNaive_Request * data =
      (ffw_collision_checker__srv__SolveCollisionNaive_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ffw_collision_checker__srv__SolveCollisionNaive_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ffw_collision_checker__srv__SolveCollisionNaive_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ffw_collision_checker__srv__SolveCollisionNaive_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ffw_collision_checker__srv__SolveCollisionNaive_Response__init(ffw_collision_checker__srv__SolveCollisionNaive_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accept
  // error
  return true;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Response__fini(ffw_collision_checker__srv__SolveCollisionNaive_Response * msg)
{
  if (!msg) {
    return;
  }
  // accept
  // error
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Response__are_equal(const ffw_collision_checker__srv__SolveCollisionNaive_Response * lhs, const ffw_collision_checker__srv__SolveCollisionNaive_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accept
  if (lhs->accept != rhs->accept) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  return true;
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Response__copy(
  const ffw_collision_checker__srv__SolveCollisionNaive_Response * input,
  ffw_collision_checker__srv__SolveCollisionNaive_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accept
  output->accept = input->accept;
  // error
  output->error = input->error;
  return true;
}

ffw_collision_checker__srv__SolveCollisionNaive_Response *
ffw_collision_checker__srv__SolveCollisionNaive_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Response * msg = (ffw_collision_checker__srv__SolveCollisionNaive_Response *)allocator.allocate(sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Response));
  bool success = ffw_collision_checker__srv__SolveCollisionNaive_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Response__destroy(ffw_collision_checker__srv__SolveCollisionNaive_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ffw_collision_checker__srv__SolveCollisionNaive_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__init(ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Response * data = NULL;

  if (size) {
    data = (ffw_collision_checker__srv__SolveCollisionNaive_Response *)allocator.zero_allocate(size, sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ffw_collision_checker__srv__SolveCollisionNaive_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ffw_collision_checker__srv__SolveCollisionNaive_Response__fini(&data[i - 1]);
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
ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__fini(ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * array)
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
      ffw_collision_checker__srv__SolveCollisionNaive_Response__fini(&array->data[i]);
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

ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence *
ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * array = (ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence *)allocator.allocate(sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__destroy(ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__are_equal(const ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * lhs, const ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ffw_collision_checker__srv__SolveCollisionNaive_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__copy(
  const ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * input,
  ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ffw_collision_checker__srv__SolveCollisionNaive_Response * data =
      (ffw_collision_checker__srv__SolveCollisionNaive_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ffw_collision_checker__srv__SolveCollisionNaive_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ffw_collision_checker__srv__SolveCollisionNaive_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ffw_collision_checker__srv__SolveCollisionNaive_Response__copy(
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
// #include "ffw_collision_checker/srv/detail/solve_collision_naive__functions.h"

bool
ffw_collision_checker__srv__SolveCollisionNaive_Event__init(ffw_collision_checker__srv__SolveCollisionNaive_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(msg);
    return false;
  }
  // request
  if (!ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__init(&msg->request, 0)) {
    ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(msg);
    return false;
  }
  // response
  if (!ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__init(&msg->response, 0)) {
    ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(msg);
    return false;
  }
  return true;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(ffw_collision_checker__srv__SolveCollisionNaive_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__fini(&msg->request);
  // response
  ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__fini(&msg->response);
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Event__are_equal(const ffw_collision_checker__srv__SolveCollisionNaive_Event * lhs, const ffw_collision_checker__srv__SolveCollisionNaive_Event * rhs)
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
  if (!ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Event__copy(
  const ffw_collision_checker__srv__SolveCollisionNaive_Event * input,
  ffw_collision_checker__srv__SolveCollisionNaive_Event * output)
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
  if (!ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

ffw_collision_checker__srv__SolveCollisionNaive_Event *
ffw_collision_checker__srv__SolveCollisionNaive_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Event * msg = (ffw_collision_checker__srv__SolveCollisionNaive_Event *)allocator.allocate(sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Event));
  bool success = ffw_collision_checker__srv__SolveCollisionNaive_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Event__destroy(ffw_collision_checker__srv__SolveCollisionNaive_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__init(ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Event * data = NULL;

  if (size) {
    data = (ffw_collision_checker__srv__SolveCollisionNaive_Event *)allocator.zero_allocate(size, sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ffw_collision_checker__srv__SolveCollisionNaive_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(&data[i - 1]);
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
ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__fini(ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * array)
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
      ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(&array->data[i]);
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

ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence *
ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * array = (ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence *)allocator.allocate(sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__destroy(ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__are_equal(const ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * lhs, const ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ffw_collision_checker__srv__SolveCollisionNaive_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence__copy(
  const ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * input,
  ffw_collision_checker__srv__SolveCollisionNaive_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ffw_collision_checker__srv__SolveCollisionNaive_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ffw_collision_checker__srv__SolveCollisionNaive_Event * data =
      (ffw_collision_checker__srv__SolveCollisionNaive_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ffw_collision_checker__srv__SolveCollisionNaive_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ffw_collision_checker__srv__SolveCollisionNaive_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ffw_collision_checker__srv__SolveCollisionNaive_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
