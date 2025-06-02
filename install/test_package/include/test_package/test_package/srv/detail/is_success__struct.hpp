// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from test_package:srv/IsSuccess.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__STRUCT_HPP_
#define TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__test_package__srv__IsSuccess_Request __attribute__((deprecated))
#else
# define DEPRECATED__test_package__srv__IsSuccess_Request __declspec(deprecated)
#endif

namespace test_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IsSuccess_Request_
{
  using Type = IsSuccess_Request_<ContainerAllocator>;

  explicit IsSuccess_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->button_type = "";
    }
  }

  explicit IsSuccess_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : button_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->button_type = "";
    }
  }

  // field types and members
  using _button_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _button_type_type button_type;

  // setters for named parameter idiom
  Type & set__button_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->button_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package::srv::IsSuccess_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::srv::IsSuccess_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::srv::IsSuccess_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::srv::IsSuccess_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__srv__IsSuccess_Request
    std::shared_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__srv__IsSuccess_Request
    std::shared_ptr<test_package::srv::IsSuccess_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IsSuccess_Request_ & other) const
  {
    if (this->button_type != other.button_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const IsSuccess_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IsSuccess_Request_

// alias to use template instance with default allocator
using IsSuccess_Request =
  test_package::srv::IsSuccess_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace test_package


#ifndef _WIN32
# define DEPRECATED__test_package__srv__IsSuccess_Response __attribute__((deprecated))
#else
# define DEPRECATED__test_package__srv__IsSuccess_Response __declspec(deprecated)
#endif

namespace test_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IsSuccess_Response_
{
  using Type = IsSuccess_Response_<ContainerAllocator>;

  explicit IsSuccess_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit IsSuccess_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package::srv::IsSuccess_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::srv::IsSuccess_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::srv::IsSuccess_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::srv::IsSuccess_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__srv__IsSuccess_Response
    std::shared_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__srv__IsSuccess_Response
    std::shared_ptr<test_package::srv::IsSuccess_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IsSuccess_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const IsSuccess_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IsSuccess_Response_

// alias to use template instance with default allocator
using IsSuccess_Response =
  test_package::srv::IsSuccess_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace test_package

namespace test_package
{

namespace srv
{

struct IsSuccess
{
  using Request = test_package::srv::IsSuccess_Request;
  using Response = test_package::srv::IsSuccess_Response;
};

}  // namespace srv

}  // namespace test_package

#endif  // TEST_PACKAGE__SRV__DETAIL__IS_SUCCESS__STRUCT_HPP_
