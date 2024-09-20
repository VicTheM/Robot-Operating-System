// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from package_with_interfaces:srv/WhatIsThePoint.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__WHAT_IS_THE_POINT__BUILDER_HPP_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__WHAT_IS_THE_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "package_with_interfaces/srv/detail/what_is_the_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace package_with_interfaces
{

namespace srv
{

namespace builder
{

class Init_WhatIsThePoint_Request_quote
{
public:
  Init_WhatIsThePoint_Request_quote()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::package_with_interfaces::srv::WhatIsThePoint_Request quote(::package_with_interfaces::srv::WhatIsThePoint_Request::_quote_type arg)
  {
    msg_.quote = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::srv::WhatIsThePoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::srv::WhatIsThePoint_Request>()
{
  return package_with_interfaces::srv::builder::Init_WhatIsThePoint_Request_quote();
}

}  // namespace package_with_interfaces


namespace package_with_interfaces
{

namespace srv
{

namespace builder
{

class Init_WhatIsThePoint_Response_point
{
public:
  Init_WhatIsThePoint_Response_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::package_with_interfaces::srv::WhatIsThePoint_Response point(::package_with_interfaces::srv::WhatIsThePoint_Response::_point_type arg)
  {
    msg_.point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::srv::WhatIsThePoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::srv::WhatIsThePoint_Response>()
{
  return package_with_interfaces::srv::builder::Init_WhatIsThePoint_Response_point();
}

}  // namespace package_with_interfaces

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__WHAT_IS_THE_POINT__BUILDER_HPP_
