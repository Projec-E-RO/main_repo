// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from test_package:msg/ManipulatorPose.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__STRUCT_H_
#define TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'button_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ManipulatorPose in the package test_package.
typedef struct test_package__msg__ManipulatorPose
{
  rosidl_runtime_c__String button_type;
} test_package__msg__ManipulatorPose;

// Struct for a sequence of test_package__msg__ManipulatorPose.
typedef struct test_package__msg__ManipulatorPose__Sequence
{
  test_package__msg__ManipulatorPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__msg__ManipulatorPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEST_PACKAGE__MSG__DETAIL__MANIPULATOR_POSE__STRUCT_H_
