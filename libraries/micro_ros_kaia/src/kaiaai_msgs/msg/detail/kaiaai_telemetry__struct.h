// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiaai_msgs:msg/KaiaaiTelemetry.idl
// generated code does not contain a copyright notice

#ifndef KAIAAI_MSGS__MSG__DETAIL__KAIAAI_TELEMETRY__STRUCT_H_
#define KAIAAI_MSGS__MSG__DETAIL__KAIAAI_TELEMETRY__STRUCT_H_

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
// Member 'joint_pos'
// Member 'joint_vel'
// Member 'lds'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/KaiaaiTelemetry in the package kaiaai_msgs.
typedef struct kaiaai_msgs__msg__KaiaaiTelemetry
{
  builtin_interfaces__msg__Time stamp;
  uint32_t seq;
  float odom_pos_x;
  float odom_pos_y;
  float odom_pos_yaw;
  float odom_vel_x;
  float odom_vel_yaw;
  rosidl_runtime_c__float__Sequence joint_pos;
  rosidl_runtime_c__float__Sequence joint_vel;
  rosidl_runtime_c__uint8__Sequence lds;
} kaiaai_msgs__msg__KaiaaiTelemetry;

// Struct for a sequence of kaiaai_msgs__msg__KaiaaiTelemetry.
typedef struct kaiaai_msgs__msg__KaiaaiTelemetry__Sequence
{
  kaiaai_msgs__msg__KaiaaiTelemetry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiaai_msgs__msg__KaiaaiTelemetry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIAAI_MSGS__MSG__DETAIL__KAIAAI_TELEMETRY__STRUCT_H_
