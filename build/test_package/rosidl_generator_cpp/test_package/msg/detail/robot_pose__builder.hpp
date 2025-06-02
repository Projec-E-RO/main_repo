// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_
#define TEST_PACKAGE__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package/msg/detail/robot_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package
{

namespace msg
{

namespace builder
{

class Init_RobotPose_gripper_open
{
public:
  explicit Init_RobotPose_gripper_open(::test_package::msg::RobotPose & msg)
  : msg_(msg)
  {}
  ::test_package::msg::RobotPose gripper_open(::test_package::msg::RobotPose::_gripper_open_type arg)
  {
    msg_.gripper_open = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::msg::RobotPose msg_;
};

class Init_RobotPose_label
{
public:
  explicit Init_RobotPose_label(::test_package::msg::RobotPose & msg)
  : msg_(msg)
  {}
  Init_RobotPose_gripper_open label(::test_package::msg::RobotPose::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_RobotPose_gripper_open(msg_);
  }

private:
  ::test_package::msg::RobotPose msg_;
};

class Init_RobotPose_z
{
public:
  explicit Init_RobotPose_z(::test_package::msg::RobotPose & msg)
  : msg_(msg)
  {}
  Init_RobotPose_label z(::test_package::msg::RobotPose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_RobotPose_label(msg_);
  }

private:
  ::test_package::msg::RobotPose msg_;
};

class Init_RobotPose_y
{
public:
  explicit Init_RobotPose_y(::test_package::msg::RobotPose & msg)
  : msg_(msg)
  {}
  Init_RobotPose_z y(::test_package::msg::RobotPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobotPose_z(msg_);
  }

private:
  ::test_package::msg::RobotPose msg_;
};

class Init_RobotPose_x
{
public:
  Init_RobotPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotPose_y x(::test_package::msg::RobotPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotPose_y(msg_);
  }

private:
  ::test_package::msg::RobotPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::msg::RobotPose>()
{
  return test_package::msg::builder::Init_RobotPose_x();
}

}  // namespace test_package

#endif  // TEST_PACKAGE__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_
