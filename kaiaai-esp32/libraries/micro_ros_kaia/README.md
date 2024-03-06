# Micro-ROS Arduino library for Kaia.ai

[Kaia.ai](https://kaia.ai) is a platform for 3D-printable pet robots. Please sign up for an early launch invite [here](https://remake.ai).

This repo is an Arduino Micro-ROS library for [Kaia.ai](https://kaia.ai) home pet robots platform.

## Install a library release using Arduino Library Manager
- open your Arduino IDE
- select the Tools -> Manage Libraries menu
- type `kaia` in the search filter
- click Install

## Install a library release by .zip download
- Navigate to the [release section](https://github.com/kaiaai/micro_ros_arduino_kaiaai/releases), expand Assets
and download the latest `Source code (zip)` release
- Launch your Arduino IDE, open the Sketch -> Include library -> Add .ZIP Library... menu and
select the downloaded file named `micro_ros_arduino_kaiaai-2.0.7-iron.3.zip` (the actual version digits may differ)

## Install library using git
Alternatively, you can `git clone` this library as follows. This method may be useful if you need to edit library files or check out different versions of the library.
- confirm the location of your Arduino sketches by opening File -> Preferences in
your Windows Arduino IDE and noting the path "Sketchbook location" path, for example `C:\Users\YOUR-USER-NAME\Documents\Arduino`
- append `\libraries` to the sketchbook location path to get the path to your Arduino libraries,
e.g. `C:\Users\YOUR-USER-NAME\Documents\Arduino\libraries`
- make sure you have installed [Git for Windows](https://gitforwindows.org/) or a similar Windows Git tool
- run commands below in a Windows shell to clone this library to your Windows PC
```
cd %HOMEPATH%\Documents\Arduino\libraries
git clone -b iron --depth 1 https://github.com/kaiaai/micro_ros_arduino_kaiaai micro_ros_kaia
```
Now you can include this library into your sketch using `#include <micro_ros_kaia.h>`.

## Mod and rebuild Micro-ROS Arduino library for Kaia.ai
In some cases, tayloring Kaia.ai software to your particular robot may require tweaking the Kaia.ai library code in
addition to the Kaia.ai firmware - for example to add new types of Micro-ROS messages. Follow these steps
to [extend and/or adapt](https://micro.ros.org/docs/tutorials/advanced/create_new_type/) and rebuild the Kaia.ai
Arduino library on Windows for your particular robot design.
- Install Docker for your PC platform, e.g. [Docker for Windows](https://docs.docker.com/desktop/install/windows-install/) and make sure the Docker agent is running
- Install the [Micro-ROS Arduino library for Kaia.ai](https://github.com/kaiaai/micro_ros_arduino_kaia/) using the instructions above.
Let's assume you are using Arduino IDE for Windows and your Arduino libraries are stored under `C:\Users\YOUR-USER-NAME\Documents\Arduino\libraries`.
- Open a Windows command shell and run these commands to rebuild the library using the [Micro-ROS library builder](https://github.com/micro-ROS/micro_ros_arduino):
```
cd %HOMEPATH%\Documents\Arduino\libraries
git clone -b iron --depth 1 https://github.com/kaiaai/micro_ros_arduino_kaia micro_ros_kaia
docker run -it --rm -v .\micro_ros_kaia:/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:iron
```

## Hints
- You can also rebuild the library for a particular platform only, e.g. for ESP32:
```
docker run -it --rm -v .\micro_ros_kaia:/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:iron -p esp32
```

## Acknowledgements and modifications
This is a fork of [Micro-ROS Arduino library](https://github.com/micro-ROS/micro_ros_arduino)
adapted to Kaia.ai-based robots. Specifically, this adapted fork

- fixes `library_generation.sh` script to build the library correctly on Windows
- adds a `git clone` to download the library
- adds [kaiaai_msgs](https://github.com/kaiaai/kaiaai_msgs/) ROS2 package
- moves `WiFi.begin()` outside of the Micro-ROS library for cleaner and convenient code development
- tweaks colcon.meta to optimize library features, performance and memory usage for Kaia.ai applications
- sets up the library for inclusion into the Arduino Library Manager

### API tweaks
Now you can handle connecting to WiFi as you see fit, instead of Micro-ROS doing this for you. For example:
```
  WiFi.begin(ssid, passw);
  Serial.print("Connecting to WiFi ");

  unsigned long startMillis = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startMillis >= 10000) {
      Serial.println(" timed out");
      return;
    }
    Serial.print('.'); // Don't use F('.'), it crashes ESP32
    delay(500);
  }

  Serial.println(F(" connected"));
  Serial.print(F("IP "));
  Serial.println(WiFi.localIP());

  set_microros_wifi_transports("192.168.1.57", 8888); // Micro-ROS setup
```

## Change Log
v2.0.7-iron.4
- added KaiaaiTelemetry2 message
- compiler warning fix

v2.0.7-iron.3
- added version defines MICRO_ROS_KAIA_MAJOR, MICRO_ROS_KAIA_MINOR, MICRO_ROS_KAIA_PATCH, MICRO_ROS_KAIA_BUILD, MICRO_ROS_KAIA_DISTRO
- added version check macro IS_MICRO_ROS_KAIA_MIN_VERSION(major,minor,patch,build)

v2.0.7-iron.2
- fixed missing kaiaai_msg package
- increased the max number of services to 6 to support parameter server

v2.0.7-iron.1
- re-added support for CPU architectures other than ESP32