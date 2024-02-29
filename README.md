# Kaia.ai platform robot firmware
This repo contains:
- Arduino [ESP32 robot firmware](/kaiaai-esp32/) for the ESP32 breakout board
- Robot's [lower body extension module firmware](/kaiaai-pico-body/)
- Robot's [head extension module firmware](/kaiaai-pico-head/)

Please install these Arduino libraries using Arduino Library Manager before compiling the firmware:
- [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai)
- [LDS](https://github.com/kaiaai/LDS/)
- [PID_Timed](https://github.com/kaiaai/arduino_pid_timed)
- [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) including AsyncTCP, ESPAsyncTCP

## Firmware setup video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=XOc5kCE3MC0" target="_blank">
 <img src="http://img.youtube.com/vi/XOc5kCE3MC0/maxresdefault.jpg" alt="Robot Arduino firmware, ROS2/Docker PC setup instructions video" width="720" height="405" border="10" />
</a>

## Robot Configuration
Platform firmware in this repository replaces separate firmwares - one for each Kaia.ai-compatible robot model - with a single, configurable one.
Once you have uploaded firmware (and the sketch data) to your Kaia.ai-compatible robot:
- wait for your robot to enter the AP (WiFi access point) mode
  - alternatively, force your robot to enter the AP mode by pressing the ESP32 BOOT button for 10+ seconds
-  connect to your robot's WiFi
- navigate your browser (PC or mobile handset) to 192.168.4.1
- configure your robot and its WiFi connection by selecting the robot model, its laser sensor and motor models

This [blog post](https://kaia.ai/blog/arduino-platform-firmware-avaiable/) discusses the configuration in more detail.

![kaiaai_robot_configurator](https://github.com/kaiaai/firmware/assets/33589365/5961c7df-7ed7-460d-80ae-b7148ed91a66)

## Change Log
### v0.4.0 - in debug
- kaiaai_telemetry
  - switched to KaiaTelemetry2 message from KaiaTelemetry
  - publish /battery_state
  - publish /wifi_state RSSI

### v0.3.0
- added 3irobotix Delta-2A, Delta-2G
- library version dependencies
  - [LDS](https://github.com/kaiaai/LDS) v0.5.0
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) v1.2.7 (including AsyncTCP, ESPAsyncTCP)
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai/) v2.0.7-rolling.3
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_library) v1.1.2
- requires Kaia.ai ROS2 image `kaiaai/kaiaai-ros-dev:humble-02-11-2024` or `kaiaai/kaiaai-ros-dev:iron-02-11-2024`

### v0.2.0
- added LiDAR/LDS laser distance scan sensors support
  - YDLIDAR X3, X3-PRO
  - Neato XV11
  - SLAMTEC RPLIDAR A1
- library version dependencies
  - [LDS](https://github.com/kaiaai/LDS) v0.4.0
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) v1.2.7 (including AsyncTCP, ESPAsyncTCP)
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai/) v2.0.7-rolling.3
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_library) v1.1.2
- requires Kaia.ai ROS2 image `kaiaai/kaiaai-ros-dev:humble-02-05-2024` or `kaiaai/kaiaai-ros-dev:iron-02-05-2024`

### v0.1.0
- initial release
- supports sensors
  - YDLIDAR X4, X2/X2L
  - LDS02RR
- robot model configuration via web browser
- requires libraries
  - [LDS](https://github.com/kaiaai/LDS) v0.3.1
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_library) v1.1.2
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai/) 2.0.7-rolling.3
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) v1.2.7
- requires Kaia.ai ROS2 image `kaiaai/kaiaai-ros-dev:humble-01-28-2024` or `kaiaai/kaiaai-ros-dev:iron-01-28-2024`
