// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package:msg/ManipulatorPose.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__BUILDER_HPP_
#define TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package/msg/detail/manipulator_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package
{

namespace msg
{

namespace builder
{

class Init_ManipulatorPose_button_type
{
public:
  Init_ManipulatorPose_button_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::msg::ManipulatorPose button_type(::test_package::msg::ManipulatorPose::_button_type_type arg)
  {
    msg_.button_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::msg::ManipulatorPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::msg::ManipulatorPose>()
{
  return test_package::msg::builder::Init_ManipulatorPose_button_type();
}

}  // namespace test_package

#endif  // TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__BUILDER_HPP_
