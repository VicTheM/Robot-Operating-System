// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from package_with_interfaces:msg/AmazingQuote.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "package_with_interfaces/msg/detail/amazing_quote__rosidl_typesupport_introspection_c.h"
#include "package_with_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "package_with_interfaces/msg/detail/amazing_quote__functions.h"
#include "package_with_interfaces/msg/detail/amazing_quote__struct.h"


// Include directives for member types
// Member `quote`
// Member `philosopher_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  package_with_interfaces__msg__AmazingQuote__init(message_memory);
}

void package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_fini_function(void * message_memory)
{
  package_with_interfaces__msg__AmazingQuote__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_member_array[3] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(package_with_interfaces__msg__AmazingQuote, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quote",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(package_with_interfaces__msg__AmazingQuote, quote),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "philosopher_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(package_with_interfaces__msg__AmazingQuote, philosopher_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_members = {
  "package_with_interfaces__msg",  // message namespace
  "AmazingQuote",  // message name
  3,  // number of fields
  sizeof(package_with_interfaces__msg__AmazingQuote),
  package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_member_array,  // message members
  package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_init_function,  // function to initialize message memory (memory has to be allocated)
  package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_type_support_handle = {
  0,
  &package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_package_with_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, package_with_interfaces, msg, AmazingQuote)() {
  if (!package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_type_support_handle.typesupport_identifier) {
    package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &package_with_interfaces__msg__AmazingQuote__rosidl_typesupport_introspection_c__AmazingQuote_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
