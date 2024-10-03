# Kaia.ai platform robot firmware
This repo contains:
- Arduino [ESP32 robot firmware](/kaiaai-esp32/) for the ESP32 breakout board
- Robot's [lower body extension module firmware](/kaiaai-pico-body/)
- Robot's [head extension module firmware](/kaiaai-pico-head/)
- all libraries necessary to build the sketch
- ESP32 sketch [data upload tool](https://github.com/me-no-dev/arduino-esp32fs-plugin/)

Please visit the [Support Forum](https://github.com/makerspet/support/discussions/)!

List of supported LiDAR/LDS sensors is here.

## Installation and setup
- download the latest code by Code -> Download ZIP on this page
  - alternatively, download the latest or an older release by clicking Releases on this page; click on Assets -> Source code to download the firmware
- open the downloaded ZIP file
  - navigate inside the "firmware-xxx" folder
  - copy everything inside the "firmware-xxx" folder to your Arduino sketch folder. That includes both the Arduino sketch and the libraries folder.
- watch the video below to
  - install the ESP32 tool chain version 2.0.17 (NOT 3.0.x)
  - ignore Arduino IDE library installation in the video
  - build, upload firmware
  - upload sketch data
- follow the configuration instructions on this page

<a href="http://www.youtube.com/watch?feature=player_embedded&v=XOc5kCE3MC0" target="_blank">
 <img src="http://img.youtube.com/vi/XOc5kCE3MC0/maxresdefault.jpg" alt="Watch the one-time PC setup, firmware upload instructions video" width="720" height="405" border="10" />
</a>

## Configuration
Once you have uploaded firmware and the sketch data) to your Kaia.ai-compatible robot:
- wait for your robot to enter the AP (WiFi access point) mode
  - the robot enters the AP (WiFi access point) mode whenever the robot fails to connect to WiFi
- Alternatively, force your robot to enter the AP mode by performing a "factory reset":
  - Press the "EN" (reset) button on your ESP32 board.
  - Next, immediately after that (within 1 second) press the ESP32 "BOOT" button and hold it for 10+ seconds.
  - The ESP32 board LED will blink fast.
  - Release the "BOOT" button once the ESP32 board LED stops blinking.
- connect to your robot's WiFi (MAKERSPET)
- navigate your browser (PC or mobile handset) to 192.168.4.1
- configure your robot and its WiFi connection:
  - input your WiFi name, password
  - select your robot's model
  - select your robot's laser sensor from the [list of supported LiDAR/LDS](https://github.com/kaiaai/LDS)
  - select your robot's motor model
- press the "Configure and Connect" button
  - disconnect from your robot's WiFi and reconnect back to your own WiFi

This [blog post](https://kaia.ai/blog/arduino-platform-firmware-avaiable/) discusses the configuration in more detail.

![kaiaai_robot_configurator](https://github.com/kaiaai/firmware/assets/33589365/5961c7df-7ed7-460d-80ae-b7148ed91a66)

## Compatible Motors

### Brushless (BLDC) Motors
- BLDC motors with these specifications should (generally) work:
  - 9..24V voltage (higher voltage increases efficiency)
  - models JGA25-2418, JGA25-2430; 24.4mm outer diameter
  - ~190..450 no-load (max) RPM, ~140..350 rated RPM
  - JST SH 1.0mm 5-pin connector on the back (PWM, CW/CCW, VMOT, GND, FG)
  - built-in BLDC driver and encoder
  - ~0.5..1.5 Kg*cm rated torque 
- Examples
  - CHR-GM25-BL2418 24V 200RPM 270PPR
  - JGA25-BL2418 24V 245RPM 630PPR
  - CHR-GM25-BL2418 24V 260RPM 204PPR
  - JGA25-BL2418 24V 408RPM 127.8PPR
  - CHR-GM25-BL2418 24V 450RPM 120PPR

### Brushed Motors
- Brushed motors with these specifications should (generally) works
  - 9..24V voltage (higher voltage increases efficiency)
  - GA25-370 size; 24.4mm outer diameter
  - ~190..450 no-load (max) RPM, ~140..350 rated RPM
  - quadrature encoder (e.g. outputs two signals: ENC_A and ENC_B)
  - 6-pin connector (VMOT+, VMOT-, ENC_A, ENC_B, VENC+, GND)
- N20 motors
  - can be connected, but not recommended due to their low power
- a compatible motor driver is required when using brushed motors
  - TB6612FNG max 13.5V, max 1.2A average per motor; one TB6612FNG controls two motors
  - L298N supports 24V, max 2A average per motor; one L298N controls two motors
  - DRV8871 supports 24V, max 3.6A peak; one DRV8871 controls ONE motor
  - DRV8833, DRV8835 (IN/IN mode)

### Where to Purchase Motors/Components
- AliExpress
- Ebay
- Amazon
- online robotics specialty stores
  - robotshop.com
- in India
  - robu.in, zbotic.in, robokits.co.in, robocraze.com, roboticsdna.in, probots.co.in, roboindia.com, flipkart.com, digikey.in, mouser.in, in.element14.com

### Choosing a Motor
- choose a motor with a minimum rated torque of ~0.5Kg*cm (a guess). Your motor must have enough torque
  - to accelerate the weight of your robot reasonably quickly, let's say 2lbs or so
  - to drive over thick carpet, floor mats, etc.
- for the given rated torque, choose a motor with the highest RPM
  - the higher the motor RPM, the faster your robot can drive
- we recommend 24V BLDC ~400 RPM motors, e.g. JGA25-BL2418 24V 408RPM 127.8PPR
  - these motors are fast, powerful and (generally) long-lasting
- 12V BLDC ~300 RPM is the second choice
  - often 12V BLDC offer less torque at the same RPM compared to 24V BLDC motors
- if you cannot get a BLDC motor, get yourself a brushed GA25-370 motor with encoder
  - this motor is (generally) less powerful compared to the recommended BLDC motors
  - brushed motors (generally) do not last as long as BLDC motors
- if you cannot get a GA25-370 motor, you can use N20 motors as the last resort
  - N20 motors have considerably less torque compared to the recommended motors
  - However, some N20 motors with low RPM (e.g. <=100RPM) do offer sufficient torque

## Change Log

### v0.7.0
- set Maker's Pet mini as the default robot
- added more N20 motors
- set/get LDROBOT LD14P rotation speed- MAKERSPET_MINI works
- added YDLIDAR X4-PRO (not tested)

### v0.6.0
- MAKERSPET_MINI works
  - tested with YDLIDAR SCL, LDROBOT LD14P

### v0.5.0
- motor driver
  - brushed motor support: drivers TB6612FNG, LM298N, DRV8871 and others with same IN1, IN2 control input logic
  - N20 motors supported
  - quadrature encoders
  - reverse motor direction, reverse motor encoder - for wiring convenience
- web configuration
  - additional options including PID, motor drive type, motor encoder type
  - LiDAR scan frequency option
  - automatically loads values from previous configuration
- ROS properties
  - motors: get max RPM, derated max RPM, target RPM, current RPM
  - motor encoders: get current value, get/set PPR (pulses per revolution), get TPR (ticks per revolution)
  - motor PID: get/set Kp, Ki, Kd, update period, PID mode on-error vs. on-measurement
  - robot base: get model name, base diameter, tire diameter, wheel base
  - LiDAR: get current scan rate, LiDAR model, 
- code refactored into separate files for readability
  - motor controller code moved into its own library
- added support for YDLIDAR SCL
- set micro-ROS client key using ESP32 MAC for smoother micro-ROS reconnect
- refactored code into separate files

### v0.4.1
- added Delta-2A 230400 baud version (vs old 115200 baud)
- added Delta-2B
- added motor voltage to configuration
- added Camsense X1

### v0.4.0
- moved from KaiaaiTelemetry to KaiaaiTelemetry2 message
  - added battery voltage telemetry
  - added WiFi RSSI telemetry
- added LDROBOT LD14P laser distance scan sensor
- included all library dependencies in library/ to make the code self-contained 
  - do not use Arduino IDE Library manager
  - instead, just copy everything to your Arduino sketch folder
- included the ESP32 sketch data upload tool in tools/

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

### 1/21/2024
- updated to match PID_Timed v1.1.0 library
  - PID_Timed v1.1.0 replaced constant `#define` with class constants to fix namespace collisions
- added [LDS](https://github.com/kaiaai/LDS) library as dependency
  - refactoried and moved YDLIDAR X4 into LDS library
  - added support for Xiaomi 1st gen LDS02RR laser distance scan sensor
- started moving `#define` constants into CONFIG class to clean up namespace
- added motor choices
- miscellaneous cleanup

### 12/02/2023
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

## Acknowledgements
- Arduino libraries:
  - [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai)
  - [LDS](https://github.com/kaiaai/LDS/)
  - [PID_Timed](https://github.com/kaiaai/arduino_pid_timed)
  - [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv) including AsyncTCP
- ESP32 sketch [data upload tool](https://github.com/me-no-dev/arduino-esp32fs-plugin/)
