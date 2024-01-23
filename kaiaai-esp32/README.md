# Kaia.ai platform robot firmware for ESP32 master board
- TODO description, list of supported sensors, motors, LDS
- TODO link to board files
- TODO link to setup, configuration video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=XOc5kCE3MC0" target="_blank">
 <img src="http://img.youtube.com/vi/XOc5kCE3MC0/maxresdefault.jpg" alt="Watch the one-time PC setup, firmware upload instructions video" width="720" height="405" border="10" />
</a>

## Release notes

1/21/2024
- updated to match PID_Timed v1.1.0 library
  - PID_Timed v1.1.0 replaced constant `#define` with class constants to fix namespace collisions
- added [LDS](https://github.com/kaiaai/LDS) library as dependency
  - refactoried and moved YDLIDAR X4 into LDS library
  - added support for Xiaomi 1st gen LDS02RR laser distance scan sensor
- started moving `#define` constants into CONFIG class to clean up namespace
- added motor choices
- miscellaneous cleanup

12/02/2023
- BREAKING ESP32 pinout assignment change to support the newly ESP32 breakout board
  - the new ESP32 breakout board works
  - the motor pin change fixes the "motor kick" upon ESP32 hard reboot
  - the LDS pin change fixes the LDS motor enabled by ESP32 upon hard reboot
  - MOT_FG_RIGHT has changed from GPIO27 to GPIO35_IN
  - LDS_MOT_EN has changed from GPIO12_OUT to GPIO19
  - MOT_CW_LEFT has changed from GPIO32 to GPIO23
- requires micro_ros_kaia Arduino library version 2.0.7-any.3 minimum
- added ROS2 parameter server
  - works successfully
- added lds.motor_speed parameter
  - controls the laser distance sensor motor speed
  - type double; set lds.motor=0 to stop LDS motor; set lds.motor=1.0 for maximum speed
  - set lds.motor=-1.0 for LDS default motor speed
- added minimum micro_ros_kaia library version check
  - Arduino build errors out at compile time if the library version is too old
- renamed some #define symbols from YDLidar-specific to generic LDS