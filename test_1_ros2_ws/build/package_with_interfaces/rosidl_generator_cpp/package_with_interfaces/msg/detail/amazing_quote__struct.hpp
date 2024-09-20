// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from package_with_interfaces:msg/AmazingQuote.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__STRUCT_HPP_
#define PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__package_with_interfaces__msg__AmazingQuote __attribute__((deprecated))
#else
# define DEPRECATED__package_with_interfaces__msg__AmazingQuote __declspec(deprecated)
#endif

namespace package_with_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AmazingQuote_
{
  using Type = AmazingQuote_<ContainerAllocator>;

  explicit AmazingQuote_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->quote = "";
      this->philosopher_name = "";
    }
  }

  explicit AmazingQuote_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : quote(_alloc),
    philosopher_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->quote = "";
      this->philosopher_name = "";
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _quote_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _quote_type quote;
  using _philosopher_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _philosopher_name_type philosopher_name;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__quote(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->quote = _arg;
    return *this;
  }
  Type & set__philosopher_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->philosopher_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    package_with_interfaces::msg::AmazingQuote_<ContainerAllocator> *;
  using ConstRawPtr =
    const package_with_interfaces::msg::AmazingQuote_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      package_with_interfaces::msg::AmazingQuote_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      package_with_interfaces::msg::AmazingQuote_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__package_with_interfaces__msg__AmazingQuote
    std::shared_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__package_with_interfaces__msg__AmazingQuote
    std::shared_ptr<package_with_interfaces::msg::AmazingQuote_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AmazingQuote_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->quote != other.quote) {
      return false;
    }
    if (this->philosopher_name != other.philosopher_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const AmazingQuote_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AmazingQuote_

// alias to use template instance with default allocator
using AmazingQuote =
  package_with_interfaces::msg::AmazingQuote_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace package_with_interfaces

#endif  // PACKAGE_WITH_INTERFACES__MSG__DETAIL__AMAZING_QUOTE__STRUCT_HPP_
