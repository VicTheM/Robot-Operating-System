// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/AddressBook.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/address_book__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_AddressBook_phone_type
{
public:
  explicit Init_AddressBook_phone_type(::custom_interfaces::msg::AddressBook & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::AddressBook phone_type(::custom_interfaces::msg::AddressBook::_phone_type_type arg)
  {
    msg_.phone_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::AddressBook msg_;
};

class Init_AddressBook_phone_number
{
public:
  explicit Init_AddressBook_phone_number(::custom_interfaces::msg::AddressBook & msg)
  : msg_(msg)
  {}
  Init_AddressBook_phone_type phone_number(::custom_interfaces::msg::AddressBook::_phone_number_type arg)
  {
    msg_.phone_number = std::move(arg);
    return Init_AddressBook_phone_type(msg_);
  }

private:
  ::custom_interfaces::msg::AddressBook msg_;
};

class Init_AddressBook_last_name
{
public:
  explicit Init_AddressBook_last_name(::custom_interfaces::msg::AddressBook & msg)
  : msg_(msg)
  {}
  Init_AddressBook_phone_number last_name(::custom_interfaces::msg::AddressBook::_last_name_type arg)
  {
    msg_.last_name = std::move(arg);
    return Init_AddressBook_phone_number(msg_);
  }

private:
  ::custom_interfaces::msg::AddressBook msg_;
};

class Init_AddressBook_first_name
{
public:
  Init_AddressBook_first_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddressBook_last_name first_name(::custom_interfaces::msg::AddressBook::_first_name_type arg)
  {
    msg_.first_name = std::move(arg);
    return Init_AddressBook_last_name(msg_);
  }

private:
  ::custom_interfaces::msg::AddressBook msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::AddressBook>()
{
  return custom_interfaces::msg::builder::Init_AddressBook_first_name();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ADDRESS_BOOK__BUILDER_HPP_
