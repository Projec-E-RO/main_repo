// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from test_package:msg/ManipulatorPose.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__STRUCT_HPP_
#define TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__test_package__msg__ManipulatorPose __attribute__((deprecated))
#else
# define DEPRECATED__test_package__msg__ManipulatorPose __declspec(deprecated)
#endif

namespace test_package
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ManipulatorPose_
{
  using Type = ManipulatorPose_<ContainerAllocator>;

  explicit ManipulatorPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->button_type = "";
    }
  }

  explicit ManipulatorPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    test_package::msg::ManipulatorPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::msg::ManipulatorPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::msg::ManipulatorPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::msg::ManipulatorPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__msg__ManipulatorPose
    std::shared_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__msg__ManipulatorPose
    std::shared_ptr<test_package::msg::ManipulatorPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManipulatorPose_ & other) const
  {
    if (this->button_type != other.button_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManipulatorPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManipulatorPose_

// alias to use template instance with default allocator
using ManipulatorPose =
  test_package::msg::ManipulatorPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace test_package

#endif  // TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__STRUCT_HPP_
