# Kaia.ai platform robot firmware
This repo contains:
- Arduino [ESP32 robot firmware](/kaiaai-esp32/) for the ESP32 breakout board
- Robot's [lower body extension module firmware](/kaiaai-pico-body/)
- Robot's [head extension module firmware](/kaiaai-pico-head/)

TODO how boards are connected

## Robot Configuration
Platform firmware in this repository replaces having separate firmwares - one for each Kaia.ai-compatible robot model - with just one, but configurable.
Once you have uploaded firmware (and the sketch data) to your Kaia.ai-compatible robot:
- wait for your robot to enter the AP (WiFi access point) mode
  - alternatively, force your robot to enter the AP mode by pressing the ESP32 BOOT button for 10+ seconds
-  connect to your robot's WiFi
- navigate your browser (PC or mobile handset) to 192.168.4.1
- configure your robot and its WiFi connection by selecting the robot model, its laser sensor and motor models

This [blog post](https://kaia.ai/blog/arduino-platform-firmware-avaiable/) discusses the configuration in more detail.

![kaiaai_robot_configurator](https://github.com/kaiaai/firmware/assets/33589365/5961c7df-7ed7-460d-80ae-b7148ed91a66)
