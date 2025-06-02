// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from test_package:msg/ManipulatorPose.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__TRAITS_HPP_
#define TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "test_package/msg/detail/manipulator_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace test_package
{

namespace msg
{

inline void to_flow_style_yaml(
  const ManipulatorPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: button_type
  {
    out << "button_type: ";
    rosidl_generator_traits::value_to_yaml(msg.button_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManipulatorPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: button_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_type: ";
    rosidl_generator_traits::value_to_yaml(msg.button_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManipulatorPose & msg, bool use_flow_style = false)
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
  const test_package::msg::ManipulatorPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_package::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_package::msg::to_yaml() instead")]]
inline std::string to_yaml(const test_package::msg::ManipulatorPose & msg)
{
  return test_package::msg::to_yaml(msg);
}

template<>
inline const char * data_type<test_package::msg::ManipulatorPose>()
{
  return "test_package::msg::ManipulatorPose";
}

template<>
inline const char * name<test_package::msg::ManipulatorPose>()
{
  return "test_package/msg/ManipulatorPose";
}

template<>
struct has_fixed_size<test_package::msg::ManipulatorPose>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<test_package::msg::ManipulatorPose>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<test_package::msg::ManipulatorPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__TRAITS_HPP_
