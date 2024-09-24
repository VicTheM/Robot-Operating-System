// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from package_with_interfaces:srv/WhatIsThePoint.idl
// generated code does not contain a copyright notice
#include "package_with_interfaces/srv/detail/what_is_the_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `quote`
#include "package_with_interfaces/msg/detail/amazing_quote__functions.h"

bool
package_with_interfaces__srv__WhatIsThePoint_Request__init(package_with_interfaces__srv__WhatIsThePoint_Request * msg)
{
  if (!msg) {
    return false;
  }
  // quote
  if (!package_with_interfaces__msg__AmazingQuote__init(&msg->quote)) {
    package_with_interfaces__srv__WhatIsThePoint_Request__fini(msg);
    return false;
  }
  return true;
}

void
package_with_interfaces__srv__WhatIsThePoint_Request__fini(package_with_interfaces__srv__WhatIsThePoint_Request * msg)
{
  if (!msg) {
    return;
  }
  // quote
  package_with_interfaces__msg__AmazingQuote__fini(&msg->quote);
}

bool
package_with_interfaces__srv__WhatIsThePoint_Request__are_equal(const package_with_interfaces__srv__WhatIsThePoint_Request * lhs, const package_with_interfaces__srv__WhatIsThePoint_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // quote
  if (!package_with_interfaces__msg__AmazingQuote__are_equal(
      &(lhs->quote), &(rhs->quote)))
  {
    return false;
  }
  return true;
}

bool
package_with_interfaces__srv__WhatIsThePoint_Request__copy(
  const package_with_interfaces__srv__WhatIsThePoint_Request * input,
  package_with_interfaces__srv__WhatIsThePoint_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // quote
  if (!package_with_interfaces__msg__AmazingQuote__copy(
      &(input->quote), &(output->quote)))
  {
    return false;
  }
  return true;
}

package_with_interfaces__srv__WhatIsThePoint_Request *
package_with_interfaces__srv__WhatIsThePoint_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__WhatIsThePoint_Request * msg = (package_with_interfaces__srv__WhatIsThePoint_Request *)allocator.allocate(sizeof(package_with_interfaces__srv__WhatIsThePoint_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(package_with_interfaces__srv__WhatIsThePoint_Request));
  bool success = package_with_interfaces__srv__WhatIsThePoint_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
package_with_interfaces__srv__WhatIsThePoint_Request__destroy(package_with_interfaces__srv__WhatIsThePoint_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    package_with_interfaces__srv__WhatIsThePoint_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__init(package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__WhatIsThePoint_Request * data = NULL;

  if (size) {
    data = (package_with_interfaces__srv__WhatIsThePoint_Request *)allocator.zero_allocate(size, sizeof(package_with_interfaces__srv__WhatIsThePoint_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = package_with_interfaces__srv__WhatIsThePoint_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        package_with_interfaces__srv__WhatIsThePoint_Request__fini(&data[i - 1]);
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
package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__fini(package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * array)
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
      package_with_interfaces__srv__WhatIsThePoint_Request__fini(&array->data[i]);
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

package_with_interfaces__srv__WhatIsThePoint_Request__Sequence *
package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * array = (package_with_interfaces__srv__WhatIsThePoint_Request__Sequence *)allocator.allocate(sizeof(package_with_interfaces__srv__WhatIsThePoint_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__destroy(package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__are_equal(const package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * lhs, const package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!package_with_interfaces__srv__WhatIsThePoint_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
package_with_interfaces__srv__WhatIsThePoint_Request__Sequence__copy(
  const package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * input,
  package_with_interfaces__srv__WhatIsThePoint_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(package_with_interfaces__srv__WhatIsThePoint_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    package_with_interfaces__srv__WhatIsThePoint_Request * data =
      (package_with_interfaces__srv__WhatIsThePoint_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!package_with_interfaces__srv__WhatIsThePoint_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          package_with_interfaces__srv__WhatIsThePoint_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!package_with_interfaces__srv__WhatIsThePoint_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `point`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
package_with_interfaces__srv__WhatIsThePoint_Response__init(package_with_interfaces__srv__WhatIsThePoint_Response * msg)
{
  if (!msg) {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__init(&msg->point)) {
    package_with_interfaces__srv__WhatIsThePoint_Response__fini(msg);
    return false;
  }
  return true;
}

void
package_with_interfaces__srv__WhatIsThePoint_Response__fini(package_with_interfaces__srv__WhatIsThePoint_Response * msg)
{
  if (!msg) {
    return;
  }
  // point
  geometry_msgs__msg__Point__fini(&msg->point);
}

bool
package_with_interfaces__srv__WhatIsThePoint_Response__are_equal(const package_with_interfaces__srv__WhatIsThePoint_Response * lhs, const package_with_interfaces__srv__WhatIsThePoint_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->point), &(rhs->point)))
  {
    return false;
  }
  return true;
}

bool
package_with_interfaces__srv__WhatIsThePoint_Response__copy(
  const package_with_interfaces__srv__WhatIsThePoint_Response * input,
  package_with_interfaces__srv__WhatIsThePoint_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__copy(
      &(input->point), &(output->point)))
  {
    return false;
  }
  return true;
}

package_with_interfaces__srv__WhatIsThePoint_Response *
package_with_interfaces__srv__WhatIsThePoint_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__WhatIsThePoint_Response * msg = (package_with_interfaces__srv__WhatIsThePoint_Response *)allocator.allocate(sizeof(package_with_interfaces__srv__WhatIsThePoint_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(package_with_interfaces__srv__WhatIsThePoint_Response));
  bool success = package_with_interfaces__srv__WhatIsThePoint_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
package_with_interfaces__srv__WhatIsThePoint_Response__destroy(package_with_interfaces__srv__WhatIsThePoint_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    package_with_interfaces__srv__WhatIsThePoint_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__init(package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__WhatIsThePoint_Response * data = NULL;

  if (size) {
    data = (package_with_interfaces__srv__WhatIsThePoint_Response *)allocator.zero_allocate(size, sizeof(package_with_interfaces__srv__WhatIsThePoint_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = package_with_interfaces__srv__WhatIsThePoint_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        package_with_interfaces__srv__WhatIsThePoint_Response__fini(&data[i - 1]);
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
package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__fini(package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * array)
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
      package_with_interfaces__srv__WhatIsThePoint_Response__fini(&array->data[i]);
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

package_with_interfaces__srv__WhatIsThePoint_Response__Sequence *
package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * array = (package_with_interfaces__srv__WhatIsThePoint_Response__Sequence *)allocator.allocate(sizeof(package_with_interfaces__srv__WhatIsThePoint_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__destroy(package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__are_equal(const package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * lhs, const package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!package_with_interfaces__srv__WhatIsThePoint_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
package_with_interfaces__srv__WhatIsThePoint_Response__Sequence__copy(
  const package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * input,
  package_with_interfaces__srv__WhatIsThePoint_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(package_with_interfaces__srv__WhatIsThePoint_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    package_with_interfaces__srv__WhatIsThePoint_Response * data =
      (package_with_interfaces__srv__WhatIsThePoint_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!package_with_interfaces__srv__WhatIsThePoint_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          package_with_interfaces__srv__WhatIsThePoint_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!package_with_interfaces__srv__WhatIsThePoint_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
