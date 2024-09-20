// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from package_with_interfaces:msg/AmazingQuote.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__FUNCTIONS_H_
#define PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "package_with_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "package_with_interfaces/msg/detail/amazing_quote__struct.h"

/// Initialize msg/AmazingQuote message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * package_with_interfaces__msg__AmazingQuote
 * )) before or use
 * package_with_interfaces__msg__AmazingQuote__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
bool
package_with_interfaces__msg__AmazingQuote__init(package_with_interfaces__msg__AmazingQuote * msg);

/// Finalize msg/AmazingQuote message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
void
package_with_interfaces__msg__AmazingQuote__fini(package_with_interfaces__msg__AmazingQuote * msg);

/// Create msg/AmazingQuote message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * package_with_interfaces__msg__AmazingQuote__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
package_with_interfaces__msg__AmazingQuote *
package_with_interfaces__msg__AmazingQuote__create();

/// Destroy msg/AmazingQuote message.
/**
 * It calls
 * package_with_interfaces__msg__AmazingQuote__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
void
package_with_interfaces__msg__AmazingQuote__destroy(package_with_interfaces__msg__AmazingQuote * msg);

/// Check for msg/AmazingQuote message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
bool
package_with_interfaces__msg__AmazingQuote__are_equal(const package_with_interfaces__msg__AmazingQuote * lhs, const package_with_interfaces__msg__AmazingQuote * rhs);

/// Copy a msg/AmazingQuote message.
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
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
bool
package_with_interfaces__msg__AmazingQuote__copy(
  const package_with_interfaces__msg__AmazingQuote * input,
  package_with_interfaces__msg__AmazingQuote * output);

/// Initialize array of msg/AmazingQuote messages.
/**
 * It allocates the memory for the number of elements and calls
 * package_with_interfaces__msg__AmazingQuote__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
bool
package_with_interfaces__msg__AmazingQuote__Sequence__init(package_with_interfaces__msg__AmazingQuote__Sequence * array, size_t size);

/// Finalize array of msg/AmazingQuote messages.
/**
 * It calls
 * package_with_interfaces__msg__AmazingQuote__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
void
package_with_interfaces__msg__AmazingQuote__Sequence__fini(package_with_interfaces__msg__AmazingQuote__Sequence * array);

/// Create array of msg/AmazingQuote messages.
/**
 * It allocates the memory for the array and calls
 * package_with_interfaces__msg__AmazingQuote__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
package_with_interfaces__msg__AmazingQuote__Sequence *
package_with_interfaces__msg__AmazingQuote__Sequence__create(size_t size);

/// Destroy array of msg/AmazingQuote messages.
/**
 * It calls
 * package_with_interfaces__msg__AmazingQuote__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
void
package_with_interfaces__msg__AmazingQuote__Sequence__destroy(package_with_interfaces__msg__AmazingQuote__Sequence * array);

/// Check for msg/AmazingQuote message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
bool
package_with_interfaces__msg__AmazingQuote__Sequence__are_equal(const package_with_interfaces__msg__AmazingQuote__Sequence * lhs, const package_with_interfaces__msg__AmazingQuote__Sequence * rhs);

/// Copy an array of msg/AmazingQuote messages.
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
ROSIDL_GENERATOR_C_PUBLIC_package_with_interfaces
bool
package_with_interfaces__msg__AmazingQuote__Sequence__copy(
  const package_with_interfaces__msg__AmazingQuote__Sequence * input,
  package_with_interfaces__msg__AmazingQuote__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__FUNCTIONS_H_
