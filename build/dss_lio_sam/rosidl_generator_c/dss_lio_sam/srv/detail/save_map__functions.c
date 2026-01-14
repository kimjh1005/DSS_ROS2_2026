// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dss_lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice
#include "dss_lio_sam/srv/detail/save_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `destination`
#include "rosidl_runtime_c/string_functions.h"

bool
dss_lio_sam__srv__SaveMap_Request__init(dss_lio_sam__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // resolution
  // destination
  if (!rosidl_runtime_c__String__init(&msg->destination)) {
    dss_lio_sam__srv__SaveMap_Request__fini(msg);
    return false;
  }
  return true;
}

void
dss_lio_sam__srv__SaveMap_Request__fini(dss_lio_sam__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // resolution
  // destination
  rosidl_runtime_c__String__fini(&msg->destination);
}

bool
dss_lio_sam__srv__SaveMap_Request__are_equal(const dss_lio_sam__srv__SaveMap_Request * lhs, const dss_lio_sam__srv__SaveMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // destination
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->destination), &(rhs->destination)))
  {
    return false;
  }
  return true;
}

bool
dss_lio_sam__srv__SaveMap_Request__copy(
  const dss_lio_sam__srv__SaveMap_Request * input,
  dss_lio_sam__srv__SaveMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // resolution
  output->resolution = input->resolution;
  // destination
  if (!rosidl_runtime_c__String__copy(
      &(input->destination), &(output->destination)))
  {
    return false;
  }
  return true;
}

dss_lio_sam__srv__SaveMap_Request *
dss_lio_sam__srv__SaveMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_lio_sam__srv__SaveMap_Request * msg = (dss_lio_sam__srv__SaveMap_Request *)allocator.allocate(sizeof(dss_lio_sam__srv__SaveMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dss_lio_sam__srv__SaveMap_Request));
  bool success = dss_lio_sam__srv__SaveMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dss_lio_sam__srv__SaveMap_Request__destroy(dss_lio_sam__srv__SaveMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dss_lio_sam__srv__SaveMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dss_lio_sam__srv__SaveMap_Request__Sequence__init(dss_lio_sam__srv__SaveMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_lio_sam__srv__SaveMap_Request * data = NULL;

  if (size) {
    data = (dss_lio_sam__srv__SaveMap_Request *)allocator.zero_allocate(size, sizeof(dss_lio_sam__srv__SaveMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dss_lio_sam__srv__SaveMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dss_lio_sam__srv__SaveMap_Request__fini(&data[i - 1]);
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
dss_lio_sam__srv__SaveMap_Request__Sequence__fini(dss_lio_sam__srv__SaveMap_Request__Sequence * array)
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
      dss_lio_sam__srv__SaveMap_Request__fini(&array->data[i]);
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

dss_lio_sam__srv__SaveMap_Request__Sequence *
dss_lio_sam__srv__SaveMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_lio_sam__srv__SaveMap_Request__Sequence * array = (dss_lio_sam__srv__SaveMap_Request__Sequence *)allocator.allocate(sizeof(dss_lio_sam__srv__SaveMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dss_lio_sam__srv__SaveMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dss_lio_sam__srv__SaveMap_Request__Sequence__destroy(dss_lio_sam__srv__SaveMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dss_lio_sam__srv__SaveMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dss_lio_sam__srv__SaveMap_Request__Sequence__are_equal(const dss_lio_sam__srv__SaveMap_Request__Sequence * lhs, const dss_lio_sam__srv__SaveMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dss_lio_sam__srv__SaveMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dss_lio_sam__srv__SaveMap_Request__Sequence__copy(
  const dss_lio_sam__srv__SaveMap_Request__Sequence * input,
  dss_lio_sam__srv__SaveMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dss_lio_sam__srv__SaveMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dss_lio_sam__srv__SaveMap_Request * data =
      (dss_lio_sam__srv__SaveMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dss_lio_sam__srv__SaveMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dss_lio_sam__srv__SaveMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dss_lio_sam__srv__SaveMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dss_lio_sam__srv__SaveMap_Response__init(dss_lio_sam__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
dss_lio_sam__srv__SaveMap_Response__fini(dss_lio_sam__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
dss_lio_sam__srv__SaveMap_Response__are_equal(const dss_lio_sam__srv__SaveMap_Response * lhs, const dss_lio_sam__srv__SaveMap_Response * rhs)
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
dss_lio_sam__srv__SaveMap_Response__copy(
  const dss_lio_sam__srv__SaveMap_Response * input,
  dss_lio_sam__srv__SaveMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

dss_lio_sam__srv__SaveMap_Response *
dss_lio_sam__srv__SaveMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_lio_sam__srv__SaveMap_Response * msg = (dss_lio_sam__srv__SaveMap_Response *)allocator.allocate(sizeof(dss_lio_sam__srv__SaveMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dss_lio_sam__srv__SaveMap_Response));
  bool success = dss_lio_sam__srv__SaveMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dss_lio_sam__srv__SaveMap_Response__destroy(dss_lio_sam__srv__SaveMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dss_lio_sam__srv__SaveMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dss_lio_sam__srv__SaveMap_Response__Sequence__init(dss_lio_sam__srv__SaveMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_lio_sam__srv__SaveMap_Response * data = NULL;

  if (size) {
    data = (dss_lio_sam__srv__SaveMap_Response *)allocator.zero_allocate(size, sizeof(dss_lio_sam__srv__SaveMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dss_lio_sam__srv__SaveMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dss_lio_sam__srv__SaveMap_Response__fini(&data[i - 1]);
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
dss_lio_sam__srv__SaveMap_Response__Sequence__fini(dss_lio_sam__srv__SaveMap_Response__Sequence * array)
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
      dss_lio_sam__srv__SaveMap_Response__fini(&array->data[i]);
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

dss_lio_sam__srv__SaveMap_Response__Sequence *
dss_lio_sam__srv__SaveMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_lio_sam__srv__SaveMap_Response__Sequence * array = (dss_lio_sam__srv__SaveMap_Response__Sequence *)allocator.allocate(sizeof(dss_lio_sam__srv__SaveMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dss_lio_sam__srv__SaveMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dss_lio_sam__srv__SaveMap_Response__Sequence__destroy(dss_lio_sam__srv__SaveMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dss_lio_sam__srv__SaveMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dss_lio_sam__srv__SaveMap_Response__Sequence__are_equal(const dss_lio_sam__srv__SaveMap_Response__Sequence * lhs, const dss_lio_sam__srv__SaveMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dss_lio_sam__srv__SaveMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dss_lio_sam__srv__SaveMap_Response__Sequence__copy(
  const dss_lio_sam__srv__SaveMap_Response__Sequence * input,
  dss_lio_sam__srv__SaveMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dss_lio_sam__srv__SaveMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dss_lio_sam__srv__SaveMap_Response * data =
      (dss_lio_sam__srv__SaveMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dss_lio_sam__srv__SaveMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dss_lio_sam__srv__SaveMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dss_lio_sam__srv__SaveMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
