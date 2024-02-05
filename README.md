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

### v0.1.0
- initial release
- list of supported LiDAR/LDS laser distance scan sensors
  - YDLIDAR X4, X3, X3-PRO, X2/X2L
  - Xiaomi Mi 1st gen LDS02RR
  - Neato XV11
  - RPLIDAR A1
- library verion dependencies
  - [LDS](https://github.com/kaiaai/LDS) v0.4.0
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) v1.2.7 (including AsyncTCP, ESPAsyncTCP)
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai/) v2.0.7-rolling.3
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_library) v1.1.2
- use Kaia.ai ROS2 image dated 02/04/2024
