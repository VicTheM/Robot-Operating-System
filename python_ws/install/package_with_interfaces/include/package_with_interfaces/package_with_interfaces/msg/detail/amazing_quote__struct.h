// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from package_with_interfaces:msg/AmazingQuote.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__STRUCT_H_
#define PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'quote'
// Member 'philosopher_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AmazingQuote in the package package_with_interfaces.
/**
  * I have done this in another workspace, so I simply copy-pasted
  * the code from the python tutorial
  *
  * AmazingQuote.msg from https://ros2-tutorial.readthedocs.io
  * An inspirational quote a day keeps the therapist away
 */
typedef struct package_with_interfaces__msg__AmazingQuote
{
  int32_t id;
  rosidl_runtime_c__String quote;
  rosidl_runtime_c__String philosopher_name;
} package_with_interfaces__msg__AmazingQuote;

// Struct for a sequence of package_with_interfaces__msg__AmazingQuote.
typedef struct package_with_interfaces__msg__AmazingQuote__Sequence
{
  package_with_interfaces__msg__AmazingQuote * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} package_with_interfaces__msg__AmazingQuote__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__STRUCT_H_
