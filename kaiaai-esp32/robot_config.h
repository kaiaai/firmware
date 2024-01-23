// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Choose your LDS
//#define LDS_YDLIDAR_X4_
#define LDS_LDS02RR_

class CONFIG {
public:
  // ESP32 pin assignment
  static const uint8_t LED_PIN = 2; // ESP32 on-board LED
  static const uint8_t LDS_MOTOR_PWM_PIN = 15; // LDS motor speed control using PWM
  static const uint8_t LDS_MOTOR_EN_PIN = 19; // LDS motor enable pin (was 12)
  static const uint8_t BAT_ADC_PIN = 36;
  static const uint8_t MOT_PWM_LEFT_PIN = 33;
  static const uint8_t MOT_CW_LEFT_PIN = 23; // was 32
  static const uint8_t MOT_FG_LEFT_PIN = 34;
  static const uint8_t MOT_PWM_RIGHT_PIN = 13;
  static const uint8_t MOT_CW_RIGHT_PIN = 25;
  static const uint8_t MOT_FG_RIGHT_PIN = 35; // was 27

  static const uint8_t RESET_SETTINGS_HOLD_SEC = 10; // Hold BOOT button to reset WiFi
  static const int LDS_MOTOR_SPEED_DEFAULT = -1; // tristate YDLidar X4 SCTP pin for default motor speed
  static const uint8_t LDS_MOTOR_PWM_CHANNEL = 2; // ESP32 PWM channel for LDS motor speed control

// Chihai Motor CHR-GM25-BL2418 24V 200RPM max
//#define MOTOR_MAX_RPM              200  // no-load
//#define MOTOR_RATED_RPM            145
//#define MOTOR_GEAR_RATIO           45.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution

// Chihai Motor CHR-GM25-BL2418 24V 260RPM max
//#define MOTOR_MAX_RPM              200  // no-load
//#define MOTOR_RATED_RPM            190
//#define MOTOR_GEAR_RATIO           34.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution

// Chihai Motor CHR-GM25-BL2418 24V 450 RPM max
//#define MOTOR_MAX_RPM              450  // no-load
//#define MOTOR_RATED_RPM            325
//#define MOTOR_GEAR_RATIO           20.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution

// Far Along JGA25-BL2418 24V 245RPM max
//#define MOTOR_MAX_RPM              245  // no-load
//#define MOTOR_RATED_RPM            185
//#define MOTOR_GEAR_RATIO           35.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          18   // pulses per revolution

// AliExpress China motor store JGA25-BL2418 24V 408RPM max
//#define MOTOR_MAX_RPM              408  // no-load
//#define MOTOR_RATED_RPM            308
//#define MOTOR_GEAR_RATIO           21.3 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution; TODO check  


public: // properties set using browser GUI
  static inline float wheel_dia = 67.0; // meters
  static inline float wheel_base = 159.063; // meters
  static inline float max_wheel_accel = 2.0; // m2/sec
  static inline String robot_model_name = "MAKERSPET_LOKI";

public: // Misc constants
    enum error_blink_count { // ESP32 blinks when firmware init fails
    ERR_WIFI_CONN = 1,
    ERR_LDS_START = 2,
    ERR_UROS_AGENT_CONN = 3,
    ERR_WIFI_LOST = 4,
    ERR_UROS_INIT = 5,
    ERR_UROS_NODE = 6,
    ERR_UROS_PUBSUB = 7,
    ERR_UROS_EXEC = 8,
    ERR_UROS_TIME_SYNC = 9,
    ERR_UROS_SPIN = 10,
    ERR_UROS_PARAM = 11,
    ERR_SPIFFS_INIT = 12,
  };

  static const uint8_t ERR_REBOOT_BLINK_CYCLES = 3; // Blink out an error a few times, then reboot
  static const uint32_t LONG_BLINK_MS = 1000;
  static const uint32_t LONG_BLINK_PAUSE_MS = 2000;
  static const uint32_t SHORT_BLINK_MS = 200;
  static const uint32_t SHORT_BLINK_PAUSE_MS = 500;

  static const uint32_t SPIN_TELEM_STATS = 100;

  // Micro-ROS config
  static const uint32_t UROS_CLIENT_KEY = 0xCA1AA100;
  static inline const String UROS_TELEM_TOPIC_NAME = "telemetry";
  static inline const String UROS_LOG_TOPIC_NAME = "rosout";
  static inline const String UROS_CMD_VEL_TOPIC_NAME = "cmd_vel";
  //#define UROS_NODE_NAME UROS_ROBOT_MODEL
  static const uint32_t UROS_PING_PUB_PERIOD_MS = 10000;
  static const uint32_t UROS_TELEM_PUB_PERIOD_MS = 50;
  static const uint32_t UROS_TIME_SYNC_TIMEOUT_MS = 1000;
  static inline const String UROS_PARAM_LDS_MOTOR_SPEED = "lds.motor_speed";

  static const uint16_t LDS_BUF_LEN = 400;
  static const uint32_t LDS_MOTOR_PWM_FREQ = 10000;
  static const uint8_t LDS_MOTOR_PWM_BITS = 11; // was 8
  static const uint8_t JOINTS_LEN = 2; // (MOTOR_COUNT)

  // WiFi config
  static const uint32_t WIFI_CONN_TIMEOUT_SEC = 30;

public:
  // Cache divisions; there is no hardware divider in ESP32
  float speed_diff_to_us;
  float wheel_base_recip;
  float wheel_radius;
  float wheel_perim_len_div60;
  float wheel_perim_len_div60_recip;

public:
  CONFIG() {
    recalculate();
  }
  void recalculate() {
    speed_diff_to_us = 1e6/max_wheel_accel;
    wheel_base_recip = 1/wheel_base;
    wheel_radius = wheel_dia / 2;

    wheel_perim_len_div60 = PI * wheel_dia / 60;
    wheel_perim_len_div60_recip = 1/wheel_perim_len_div60;
  }

  // Hack
  //#define SPEED_TO_RPM(SPEED_MS) (SPEED_MS*WHEEL_PERIM_LEN_DIV60_RECIP);
  float speed_to_rpm(float speed_ms) {
    return speed_ms*wheel_perim_len_div60_recip;
  }
  
  //#define RPM_TO_SPEED(RPM) (RPM*WHEEL_PERIM_LEN_DIV60);
  float rpm_to_speed(float rpm) {
    return rpm*wheel_perim_len_div60;
  }
  
  void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
    float *speed_right, float *speed_left) {
    float ang_component = speed_ang_z*wheel_base*0.5f;
    *speed_right = speed_lin_x + ang_component;
    *speed_left  = speed_lin_x - ang_component;
  }
};

#if !defined(ESP32)
  #error This code builds on ESP32 Dev module only
#endif
