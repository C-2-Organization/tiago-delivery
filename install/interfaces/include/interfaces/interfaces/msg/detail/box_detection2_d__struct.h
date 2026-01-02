// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/BoxDetection2D.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__BOX_DETECTION2_D__STRUCT_H_
#define INTERFACES__MSG__DETAIL__BOX_DETECTION2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'source'
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BoxDetection2D in the package interfaces.
typedef struct interfaces__msg__BoxDetection2D
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String source;
  rosidl_runtime_c__String label;
  float confidence;
  int32_t x1;
  int32_t y1;
  int32_t x2;
  int32_t y2;
} interfaces__msg__BoxDetection2D;

// Struct for a sequence of interfaces__msg__BoxDetection2D.
typedef struct interfaces__msg__BoxDetection2D__Sequence
{
  interfaces__msg__BoxDetection2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__BoxDetection2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__BOX_DETECTION2_D__STRUCT_H_
