// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from test_package:srv/IsSuccess.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__TRAITS_HPP_
#define TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "test_package/srv/detail/is_success__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace test_package
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsSuccess_Request & msg,
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
  const IsSuccess_Request & msg,
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

inline std::string to_yaml(const IsSuccess_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace test_package

namespace rosidl_generator_traits
{

[[deprecated("use test_package::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_package::srv::IsSuccess_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_package::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_package::srv::to_yaml() instead")]]
inline std::string to_yaml(const test_package::srv::IsSuccess_Request & msg)
{
  return test_package::srv::to_yaml(msg);
}

template<>
inline const char * data_type<test_package::srv::IsSuccess_Request>()
{
  return "test_package::srv::IsSuccess_Request";
}

template<>
inline const char * name<test_package::srv::IsSuccess_Request>()
{
  return "test_package/srv/IsSuccess_Request";
}

template<>
struct has_fixed_size<test_package::srv::IsSuccess_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<test_package::srv::IsSuccess_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<test_package::srv::IsSuccess_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace test_package
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsSuccess_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsSuccess_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsSuccess_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace test_package

namespace rosidl_generator_traits
{

[[deprecated("use test_package::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_package::srv::IsSuccess_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_package::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_package::srv::to_yaml() instead")]]
inline std::string to_yaml(const test_package::srv::IsSuccess_Response & msg)
{
  return test_package::srv::to_yaml(msg);
}

template<>
inline const char * data_type<test_package::srv::IsSuccess_Response>()
{
  return "test_package::srv::IsSuccess_Response";
}

template<>
inline const char * name<test_package::srv::IsSuccess_Response>()
{
  return "test_package/srv/IsSuccess_Response";
}

template<>
struct has_fixed_size<test_package::srv::IsSuccess_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<test_package::srv::IsSuccess_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<test_package::srv::IsSuccess_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<test_package::srv::IsSuccess>()
{
  return "test_package::srv::IsSuccess";
}

template<>
inline const char * name<test_package::srv::IsSuccess>()
{
  return "test_package/srv/IsSuccess";
}

template<>
struct has_fixed_size<test_package::srv::IsSuccess>
  : std::integral_constant<
    bool,
    has_fixed_size<test_package::srv::IsSuccess_Request>::value &&
    has_fixed_size<test_package::srv::IsSuccess_Response>::value
  >
{
};

template<>
struct has_bounded_size<test_package::srv::IsSuccess>
  : std::integral_constant<
    bool,
    has_bounded_size<test_package::srv::IsSuccess_Request>::value &&
    has_bounded_size<test_package::srv::IsSuccess_Response>::value
  >
{
};

template<>
struct is_service<test_package::srv::IsSuccess>
  : std::true_type
{
};

template<>
struct is_service_request<test_package::srv::IsSuccess_Request>
  : std::true_type
{
};

template<>
struct is_service_response<test_package::srv::IsSuccess_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__TRAITS_HPP_
