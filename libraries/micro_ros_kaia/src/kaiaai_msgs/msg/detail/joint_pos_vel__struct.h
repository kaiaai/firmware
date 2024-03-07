// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiaai_msgs:msg/JointPosVel.idl
// generated code does not contain a copyright notice

#ifndef KAIAAI_MSGS__MSG__DETAIL__JOINT_POS_VEL__STRUCT_H_
#define KAIAAI_MSGS__MSG__DETAIL__JOINT_POS_VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/JointPosVel in the package kaiaai_msgs.
typedef struct kaiaai_msgs__msg__JointPosVel
{
  float pos;
  float vel;
} kaiaai_msgs__msg__JointPosVel;

// Struct for a sequence of kaiaai_msgs__msg__JointPosVel.
typedef struct kaiaai_msgs__msg__JointPosVel__Sequence
{
  kaiaai_msgs__msg__JointPosVel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiaai_msgs__msg__JointPosVel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIAAI_MSGS__MSG__DETAIL__JOINT_POS_VEL__STRUCT_H_
