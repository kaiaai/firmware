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
#include <SPIFFS.h>

#if !defined(ESP32)
  #error This code builds on ESP32
#endif

class CONFIG {
public:
  static constexpr char* FW_VERSION = (char*)"0.4.0-iron";
  // ESP32 pin assignment
  static const uint8_t LED_PIN = 2; // ESP32 on-board LED
  static const uint8_t LDS_MOTOR_PWM_PIN = 15;
  static const uint8_t LDS_MOTOR_EN_PIN = 19;
  static const uint8_t BAT_ADC_PIN = 36;
  static const uint8_t MOT_PWM_LEFT_PIN = 33;
  static const uint8_t MOT_CW_LEFT_PIN = 23; // was 32
  static const uint8_t MOT_FG_LEFT_PIN = 34;
  static const uint8_t MOT_PWM_RIGHT_PIN = 13;
  static const uint8_t MOT_CW_RIGHT_PIN = 25;
  static const uint8_t MOT_FG_RIGHT_PIN = 35; // was 27

  static const uint32_t RESET_SETTINGS_HOLD_MS = 10000; // Hold BOOT button to reset WiFi
  static constexpr double LDS_SCAN_FREQ_DEFAULT = 0;
  static const uint8_t LDS_MOTOR_PWM_CHANNEL = 2; // ESP32 PWM channel for LDS motor speed control

  static const uint8_t BAT_ADC_MULTIPLIER = 11; // resistor divider reciprocal
  static const uint16_t BAT_PRESENT_MV_MIN = 4000;

  enum param_name_index {
    PARAM_SSID = 0,
    PARAM_PASS = 1,
    PARAM_DEST_IP = 2,
    PARAM_DEST_PORT = 3,
    PARAM_ROBOT_MODEL_NAME = 4,
    PARAM_LDS_MODEL = 5,
    PARAM_BASE_DIA_MM = 6,
    PARAM_WHEEL_BASE_MM = 7,
    PARAM_WHEEL_DIA_MM = 8,
    PARAM_MAX_WHEEL_ACCEL = 9,
    PARAM_MOTOR_MAX_RPM = 10,
    PARAM_WHEEL_PPR = 11,
    PARAM_MOTOR_VOLTAGE = 12,
    PARAM_COUNT = 13,
  };

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

protected:
  String param_value[PARAM_COUNT] = {"", "", "", "8888",
    "MAKERSPET_LOKI", "YDLIDAR X4", "204.2", "159.063", "67", "2.0", "200", "270"};
  char* PARAM_NAME[PARAM_COUNT] = {(char *)"ssid", (char *)"pass",
    (char *)"dest_ip", (char *)"dest_port", (char *)"robot_model_name",
    (char *)"lds_model", (char *)"base_dia", (char *)"wheel_base",
    (char *)"wheel_dia", (char *)"max_wheel_accel",
    (char *)"motor_max_rpm", (char *)"wheel_ppr", (char *)"motor_voltage"};

public:
  static const uint8_t ERR_REBOOT_BLINK_CYCLES = 3; // Blink out an error a few times, then reboot
  static const uint32_t LONG_BLINK_MS = 1000;
  static const uint32_t LONG_BLINK_PAUSE_MS = 2000;
  static const uint32_t SHORT_BLINK_MS = 200;
  static const uint32_t SHORT_BLINK_PAUSE_MS = 500;
  static const uint32_t SPIN_TELEM_STATS = 100;
  static constexpr float MOTOR_MAX_RPM_DERATE = 0.9;

  // Micro-ROS config
  static const uint32_t UROS_CLIENT_KEY = 0xCA1AA100;
  static constexpr char * UROS_TELEM_TOPIC_NAME = (char *)"telemetry";
  static constexpr char * UROS_LOG_TOPIC_NAME = (char *)"rosout";
  static constexpr char * UROS_CMD_VEL_TOPIC_NAME = (char *)"cmd_vel";
  //#define UROS_NODE_NAME UROS_ROBOT_MODEL
  static const uint32_t UROS_PING_PUB_PERIOD_MS = 10000;
  static const uint32_t UROS_TELEM_PUB_PERIOD_MS = 50;
  static const uint32_t UROS_TIME_SYNC_TIMEOUT_MS = 1000;
  static constexpr char * UROS_PARAM_LDS_SCAN_FREQ = (char *)"lds.scan_freq";

  static const uint16_t LDS_BUF_LEN = 400;
  static const uint32_t LDS_MOTOR_PWM_FREQ = 10000;
  static const uint8_t LDS_MOTOR_PWM_BITS = 11;
  static const uint16_t LDS_SERIAL_RX_BUF_LEN = 1024;

  static const uint32_t WIFI_CONN_TIMEOUT_MS = 30000;
  static constexpr char * PARAM_AP_WIFI_SSID = (char *) "MAKERSPET";

public:
  // Hack  
  // Cache divisions
  float speed_diff_to_us;
  float wheel_base_recip;
  float wheel_radius;
  float wheel_perim_len_div60;
  float wheel_perim_len_div60_recip;

public:
  char* const* getParamNames() {
    return PARAM_NAME;
  }

  String * getParamValues() {
    return param_value;
  }

  // Hack
  void setWheelDia(const char * wheel_dia_mm_str) {
    float wheel_dia = String(wheel_dia_mm_str).toFloat()*0.001;
    
    wheel_radius = wheel_dia * 0.5;
    wheel_perim_len_div60 = PI * wheel_dia / 60;
    wheel_perim_len_div60_recip = 1/wheel_perim_len_div60;
  }
  
  void setMaxWheelAccel(const char * max_wheel_accel_str) {
    float max_wheel_accel = String(max_wheel_accel_str).toFloat();
    speed_diff_to_us = 1e6/max_wheel_accel;
  }
  
  void setWheelBase(const char * wheel_base_mm_str) {
    float wheel_base = String(wheel_base_mm_str).toFloat()*0.001;
    wheel_base_recip = 1/wheel_base;
  }
  
  // Hack
  float speed_to_rpm(float speed_ms) {
    return speed_ms*wheel_perim_len_div60_recip;
  }
  
  float rpm_to_speed(float rpm) {
    return rpm*wheel_perim_len_div60;
  }
  
  void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
    float *speed_right, float *speed_left) {
    float ang_component = speed_ang_z*wheel_radius; //wheel_base*0.5f;
    *speed_right = speed_lin_x + ang_component;
    *speed_left  = speed_lin_x - ang_component;
  }
};
