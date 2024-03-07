// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiaai_msgs:msg/KaiaaiTelemetry2.idl
// generated code does not contain a copyright notice

#ifndef KAIAAI_MSGS__MSG__DETAIL__KAIAAI_TELEMETRY2__STRUCT_H_
#define KAIAAI_MSGS__MSG__DETAIL__KAIAAI_TELEMETRY2__STRUCT_H_

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
// Member 'joint'
#include "kaiaai_msgs/msg/detail/joint_pos_vel__struct.h"
// Member 'distance_mm'
// Member 'bumper'
// Member 'cliff'
// Member 'touch'
// Member 'lds'
#include "rosidl_runtime_c/primitives_sequence.h"

// constants for array fields with an upper bound
// joint
enum
{
  kaiaai_msgs__msg__KaiaaiTelemetry2__joint__MAX_SIZE = 15
};
// distance_mm
enum
{
  kaiaai_msgs__msg__KaiaaiTelemetry2__distance_mm__MAX_SIZE = 15
};
// bumper
enum
{
  kaiaai_msgs__msg__KaiaaiTelemetry2__bumper__MAX_SIZE = 15
};
// cliff
enum
{
  kaiaai_msgs__msg__KaiaaiTelemetry2__cliff__MAX_SIZE = 15
};
// touch
enum
{
  kaiaai_msgs__msg__KaiaaiTelemetry2__touch__MAX_SIZE = 15
};
// lds
enum
{
  kaiaai_msgs__msg__KaiaaiTelemetry2__lds__MAX_SIZE = 511
};

/// Struct defined in msg/KaiaaiTelemetry2 in the package kaiaai_msgs.
typedef struct kaiaai_msgs__msg__KaiaaiTelemetry2
{
  builtin_interfaces__msg__Time stamp;
  uint32_t seq;
  float odom_pos_x;
  float odom_pos_y;
  float odom_pos_yaw;
  float odom_vel_x;
  float odom_vel_yaw;
  kaiaai_msgs__msg__JointPosVel__Sequence joint;
  int8_t wifi_rssi_dbm;
  uint16_t battery_mv;
  rosidl_runtime_c__uint16__Sequence distance_mm;
  rosidl_runtime_c__boolean__Sequence bumper;
  rosidl_runtime_c__boolean__Sequence cliff;
  rosidl_runtime_c__uint16__Sequence touch;
  bool scan_start_hint;
  rosidl_runtime_c__uint8__Sequence lds;
} kaiaai_msgs__msg__KaiaaiTelemetry2;

// Struct for a sequence of kaiaai_msgs__msg__KaiaaiTelemetry2.
typedef struct kaiaai_msgs__msg__KaiaaiTelemetry2__Sequence
{
  kaiaai_msgs__msg__KaiaaiTelemetry2 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiaai_msgs__msg__KaiaaiTelemetry2__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIAAI_MSGS__MSG__DETAIL__KAIAAI_TELEMETRY2__STRUCT_H_
