// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from test_package:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__MSG__DETAIL__ROBOT_POSE__TRAITS_HPP_
#define TEST_PACKAGE__MSG__DETAIL__ROBOT_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "test_package/msg/detail/robot_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace test_package
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: gripper_open
  {
    out << "gripper_open: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_open, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: gripper_open
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_open: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_open, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace test_package

namespace rosidl_generator_traits
{

[[deprecated("use test_package::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_package::msg::RobotPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_package::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_package::msg::to_yaml() instead")]]
inline std::string to_yaml(const test_package::msg::RobotPose & msg)
{
  return test_package::msg::to_yaml(msg);
}

template<>
inline const char * data_type<test_package::msg::RobotPose>()
{
  return "test_package::msg::RobotPose";
}

template<>
inline const char * name<test_package::msg::RobotPose>()
{
  return "test_package/msg/RobotPose";
}

template<>
struct has_fixed_size<test_package::msg::RobotPose>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<test_package::msg::RobotPose>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<test_package::msg::RobotPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEST_PACKAGE__MSG__DETAIL__ROBOT_POSE__TRAITS_HPP_
