// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package:srv/IsSuccess.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__BUILDER_HPP_
#define TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package/srv/detail/is_success__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package
{

namespace srv
{

namespace builder
{

class Init_IsSuccess_Request_button_type
{
public:
  Init_IsSuccess_Request_button_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::srv::IsSuccess_Request button_type(::test_package::srv::IsSuccess_Request::_button_type_type arg)
  {
    msg_.button_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::srv::IsSuccess_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::srv::IsSuccess_Request>()
{
  return test_package::srv::builder::Init_IsSuccess_Request_button_type();
}

}  // namespace test_package


namespace test_package
{

namespace srv
{

namespace builder
{

class Init_IsSuccess_Response_success
{
public:
  Init_IsSuccess_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::srv::IsSuccess_Response success(::test_package::srv::IsSuccess_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::srv::IsSuccess_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::srv::IsSuccess_Response>()
{
  return test_package::srv::builder::Init_IsSuccess_Response_success();
}

}  // namespace test_package

#endif  // TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__BUILDER_HPP_
