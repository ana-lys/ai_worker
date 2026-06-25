// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice
#include "ffw_collision_checker/msg/detail/collision_check__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `distances`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `geom1_names`
// Member `geom2_names`
#include "rosidl_runtime_c/string_functions.h"

bool
ffw_collision_checker__msg__CollisionCheck__init(ffw_collision_checker__msg__CollisionCheck * msg)
{
  if (!msg) {
    return false;
  }
  // in_collision
  // distances
  if (!rosidl_runtime_c__double__Sequence__init(&msg->distances, 0)) {
    ffw_collision_checker__msg__CollisionCheck__fini(msg);
    return false;
  }
  // geom1_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->geom1_names, 0)) {
    ffw_collision_checker__msg__CollisionCheck__fini(msg);
    return false;
  }
  // geom2_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->geom2_names, 0)) {
    ffw_collision_checker__msg__CollisionCheck__fini(msg);
    return false;
  }
  return true;
}

void
ffw_collision_checker__msg__CollisionCheck__fini(ffw_collision_checker__msg__CollisionCheck * msg)
{
  if (!msg) {
    return;
  }
  // in_collision
  // distances
  rosidl_runtime_c__double__Sequence__fini(&msg->distances);
  // geom1_names
  rosidl_runtime_c__String__Sequence__fini(&msg->geom1_names);
  // geom2_names
  rosidl_runtime_c__String__Sequence__fini(&msg->geom2_names);
}

bool
ffw_collision_checker__msg__CollisionCheck__are_equal(const ffw_collision_checker__msg__CollisionCheck * lhs, const ffw_collision_checker__msg__CollisionCheck * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // in_collision
  if (lhs->in_collision != rhs->in_collision) {
    return false;
  }
  // distances
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->distances), &(rhs->distances)))
  {
    return false;
  }
  // geom1_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->geom1_names), &(rhs->geom1_names)))
  {
    return false;
  }
  // geom2_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->geom2_names), &(rhs->geom2_names)))
  {
    return false;
  }
  return true;
}

bool
ffw_collision_checker__msg__CollisionCheck__copy(
  const ffw_collision_checker__msg__CollisionCheck * input,
  ffw_collision_checker__msg__CollisionCheck * output)
{
  if (!input || !output) {
    return false;
  }
  // in_collision
  output->in_collision = input->in_collision;
  // distances
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->distances), &(output->distances)))
  {
    return false;
  }
  // geom1_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->geom1_names), &(output->geom1_names)))
  {
    return false;
  }
  // geom2_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->geom2_names), &(output->geom2_names)))
  {
    return false;
  }
  return true;
}

ffw_collision_checker__msg__CollisionCheck *
ffw_collision_checker__msg__CollisionCheck__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__msg__CollisionCheck * msg = (ffw_collision_checker__msg__CollisionCheck *)allocator.allocate(sizeof(ffw_collision_checker__msg__CollisionCheck), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ffw_collision_checker__msg__CollisionCheck));
  bool success = ffw_collision_checker__msg__CollisionCheck__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ffw_collision_checker__msg__CollisionCheck__destroy(ffw_collision_checker__msg__CollisionCheck * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ffw_collision_checker__msg__CollisionCheck__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ffw_collision_checker__msg__CollisionCheck__Sequence__init(ffw_collision_checker__msg__CollisionCheck__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__msg__CollisionCheck * data = NULL;

  if (size) {
    data = (ffw_collision_checker__msg__CollisionCheck *)allocator.zero_allocate(size, sizeof(ffw_collision_checker__msg__CollisionCheck), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ffw_collision_checker__msg__CollisionCheck__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ffw_collision_checker__msg__CollisionCheck__fini(&data[i - 1]);
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
ffw_collision_checker__msg__CollisionCheck__Sequence__fini(ffw_collision_checker__msg__CollisionCheck__Sequence * array)
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
      ffw_collision_checker__msg__CollisionCheck__fini(&array->data[i]);
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

ffw_collision_checker__msg__CollisionCheck__Sequence *
ffw_collision_checker__msg__CollisionCheck__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ffw_collision_checker__msg__CollisionCheck__Sequence * array = (ffw_collision_checker__msg__CollisionCheck__Sequence *)allocator.allocate(sizeof(ffw_collision_checker__msg__CollisionCheck__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ffw_collision_checker__msg__CollisionCheck__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ffw_collision_checker__msg__CollisionCheck__Sequence__destroy(ffw_collision_checker__msg__CollisionCheck__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ffw_collision_checker__msg__CollisionCheck__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ffw_collision_checker__msg__CollisionCheck__Sequence__are_equal(const ffw_collision_checker__msg__CollisionCheck__Sequence * lhs, const ffw_collision_checker__msg__CollisionCheck__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ffw_collision_checker__msg__CollisionCheck__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ffw_collision_checker__msg__CollisionCheck__Sequence__copy(
  const ffw_collision_checker__msg__CollisionCheck__Sequence * input,
  ffw_collision_checker__msg__CollisionCheck__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ffw_collision_checker__msg__CollisionCheck);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ffw_collision_checker__msg__CollisionCheck * data =
      (ffw_collision_checker__msg__CollisionCheck *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ffw_collision_checker__msg__CollisionCheck__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ffw_collision_checker__msg__CollisionCheck__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ffw_collision_checker__msg__CollisionCheck__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
