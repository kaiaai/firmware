#ifndef MICRO_ROS_KAIA
#define MICRO_ROS_KAIA

#define MICRO_ROS_KAIA_DISTRO iron
#define MICRO_ROS_KAIA_VERSION_MAJOR 2
#define MICRO_ROS_KAIA_VERSION_MINOR 0
#define MICRO_ROS_KAIA_VERSION_PATCH 7
#define MICRO_ROS_KAIA_VERSION_BUILD 4
#define IS_MICRO_ROS_KAIA_MIN_VERSION(major,minor,patch,build) ( \
  (MICRO_ROS_KAIA_VERSION_MAJOR >= major) && \
  (MICRO_ROS_KAIA_VERSION_MINOR >= minor) && \
  (MICRO_ROS_KAIA_VERSION_PATCH >= patch) && \
  (MICRO_ROS_KAIA_VERSION_BUILD >= build))

#include <micro_ros_arduino.h>

#endif
