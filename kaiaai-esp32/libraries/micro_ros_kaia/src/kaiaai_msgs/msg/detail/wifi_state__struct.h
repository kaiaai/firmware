// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiaai_msgs:msg/WifiState.idl
// generated code does not contain a copyright notice

#ifndef KAIAAI_MSGS__MSG__DETAIL__WIFI_STATE__STRUCT_H_
#define KAIAAI_MSGS__MSG__DETAIL__WIFI_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/WifiState in the package kaiaai_msgs.
typedef struct kaiaai_msgs__msg__WifiState
{
  builtin_interfaces__msg__Time stamp;
  float rssi_dbm;
} kaiaai_msgs__msg__WifiState;

// Struct for a sequence of kaiaai_msgs__msg__WifiState.
typedef struct kaiaai_msgs__msg__WifiState__Sequence
{
  kaiaai_msgs__msg__WifiState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiaai_msgs__msg__WifiState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIAAI_MSGS__MSG__DETAIL__WIFI_STATE__STRUCT_H_
