# Kaia.ai platform robot firmware for ESP32 master board
- TODO description, list of supported sensors, motors, LDS
- TODO link to board files
- TODO link to setup, configuration video

## Installation
- on this page click Code -> Download ZIP
- open the downloaded ZIP file, navigate inside kaiaai-esp32 folder
  - copy library folder (from inside kaiaai-esp32 folder) to your Arduino sketch folder
  - copy tools folder (from inside kaiaai-esp32 folder) to your Arduino sketch folder
  - navigate back out of the kaiaai-esp32 folder and copy kaiaai-esp32 folder to your Arduino sketch folder
- install the ESP32 tool chain, see the video below

<a href="http://www.youtube.com/watch?feature=player_embedded&v=XOc5kCE3MC0" target="_blank">
 <img src="http://img.youtube.com/vi/XOc5kCE3MC0/maxresdefault.jpg" alt="Watch the one-time PC setup, firmware upload instructions video" width="720" height="405" border="10" />
</a>

## Change Log
v0.4.0
- moved from KaiaaiTelemetry to KaiaaiTelemetry2 message
  - added battery voltage telemetry
  - added WiFi RSSI telemetry
- added LDROBOT LD14P laser distance scan sensor
- included all library dependencies in library/ to make the code self-contained 
  - do not use Arduino IDE Library manager
  - instead, move kaiaai-esp32/library into your Arduino sketch folder
- install the data upload tool by moving kaiaai-esp32/tools into your Arduino sketch folder

v0.3.0
- added 3irobotix Delta-2A, Delta-2G
- library version dependencies
  - [LDS](https://github.com/kaiaai/LDS) v0.5.0
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) v1.2.7 (including AsyncTCP, ESPAsyncTCP)
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai/) v2.0.7-rolling.3
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_library) v1.1.2
- requires Kaia.ai ROS2 image `kaiaai/kaiaai-ros-dev:humble-02-11-2024` or `kaiaai/kaiaai-ros-dev:iron-02-11-2024`

v0.2.0
- added LiDAR/LDS laser distance scan sensors support
  - YDLIDAR X3, X3-PRO
  - Neato XV11
  - RPLIDAR A1
- library version dependencies
  - [LDS](https://github.com/kaiaai/LDS) v0.4.0
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) v1.2.7 (including AsyncTCP, ESPAsyncTCP)
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai/) v2.0.7-rolling.3
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_library) v1.1.2
- requires Kaia.ai ROS2 image `kaiaai/kaiaai-ros-dev:humble-02-05-2024` or `kaiaai/kaiaai-ros-dev:iron-02-05-2024`

v0.1.0
- supports YDLIDAR X4, LDS02RR, YDLIDAR X2/X2L
- robot model configuration via web browser
- requires libraries LDS v0.3.1, PID_Timed v1.1.2, micro_ros_kaia 2.0.7-rolling.3, ESPAsyncWebSrv v1.2.7

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