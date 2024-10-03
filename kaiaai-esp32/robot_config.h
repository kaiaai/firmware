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

class CONFIG {
public:
  static constexpr char* FW_VERSION = (char*)"0.7.0-iron";

  // ESP32 pin assignment
  static const uint8_t LED_PIN = 2; // ESP32 on-board LED
  static const uint8_t BAT_ADC_PIN = 36;

  // LiDAR
  static const uint8_t LIDAR_PWM_PIN = 15;
  static const uint8_t LIDAR_EN_PIN = 19;
  
  // Brushless motors
//  static const uint8_t MOT_PWM_LEFT_PIN = 33; // weaker pulldown
  static const uint8_t MOT_PWM_LEFT_PIN = 27;
  static const uint8_t MOT_CW_LEFT_PIN = 23;
  static const uint8_t MOT_FG_LEFT_PIN = 34;

  static const uint8_t MOT_PWM_RIGHT_PIN = 25; //weaker pulldown, was 13;
  static const uint8_t MOT_CW_RIGHT_PIN = 13;  //25;
  static const uint8_t MOT_FG_RIGHT_PIN = 35;
  
  // Brushed motors
  static const uint8_t MOT_ENC_A_RIGHT_PIN = MOT_FG_RIGHT_PIN;
  static const uint8_t MOT_ENC_B_RIGHT_PIN = 26;
  static const uint8_t MOT_IN1_RIGHT_PIN = MOT_PWM_RIGHT_PIN;
  static const uint8_t MOT_IN2_RIGHT_PIN = MOT_CW_RIGHT_PIN;

  static const uint8_t MOT_ENC_A_LEFT_PIN = MOT_FG_LEFT_PIN;
  static const uint8_t MOT_ENC_B_LEFT_PIN = 32;
  static const uint8_t MOT_IN1_LEFT_PIN = MOT_PWM_LEFT_PIN;
  static const uint8_t MOT_IN2_LEFT_PIN = MOT_CW_LEFT_PIN;

  // PWM channels
  enum pwm_channel {
    MOT_PWM_LEFT_CHANNEL = 0,
    MOT_PWM_RIGHT_CHANNEL = 1,
    LIDAR_PWM_CHANNEL = 2,
  };
  static const uint16_t MOT_PWM_FREQ = 20000; // 15..25KHz
  static const uint8_t MOT_PWM_BITS = 10;

  static const uint32_t LIDAR_PWM_FREQ = 10000;
  static const uint8_t LIDAR_PWM_BITS = 11;

  static const uint8_t RESET_SETTINGS_HOLD_SEC = 3; // Hold BOOT button to reset WiFi

  static const uint16_t BAT_PRESENT_MV_MIN = 4000;

  enum param_name_index {
    PARAM_SSID,
    PARAM_PASS,
    PARAM_DEST_IP,
    PARAM_DEST_PORT,
    PARAM_ROBOT_MODEL,
    PARAM_ROBOT_MODEL_NAME,
    PARAM_LIDAR_MODEL,
    PARAM_LIDAR_SCAN_FREQ_HZ,
    PARAM_MOTOR_MODEL,
    PARAM_BASE_DIA,
    PARAM_BASE_WHEEL_TRACK,
    PARAM_BASE_WHEEL_DIA,
    PARAM_MAX_WHEEL_ACCEL,
    PARAM_MOTOR_MAX_RPM,
    PARAM_WHEEL_PPR,
    PARAM_MOTOR_VOLTAGE,
    PARAM_MOTOR_DRIVER_TYPE,
    PARAM_MOTOR_ENCODER_TYPE,
    PARAM_MOTOR_DIRECTION_REVERSED,
    PARAM_MOTOR_ENCODER_REVERSED,
    PARAM_MOTOR_PID_KP,
    PARAM_MOTOR_PID_KI,
    PARAM_MOTOR_PID_KD,
    PARAM_MOTOR_PID_MODE,
    PARAM_MOTOR_PID_PERIOD,
    PARAM_MOTOR_MAX_RPM_DERATE,
    PARAM_BATTERY_ADC_ATTENUATION,
    PARAM_COUNT,
  };

public: // Misc constants
  enum error_blink_count { // ESP32 blinks when firmware init fails
    ERR_NONE = 0,
    ERR_WIFI_CONN = 1,
    ERR_LIDAR_START = 2,
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
    "MAKERSPET_LOKI", "MAKERSPET_LOKI", "YDLIDAR X4",
    "204.2", "159.063", "12V_280RPM_234.3PPR_TB6612FNG",
    "67", "2.0", "280", "234.3", "12",
    "TB6612FNG", "ENCA_ENCB_QUAD", "NONE", "NONE",
    "0.001", "0.001", "0", "ON_MEASUREMENT", "0.03", "0.9", "11"};
  char* PARAM_NAME[PARAM_COUNT] = {(char *)"ssid", (char *)"pass",
    (char *)"dest_ip", (char *)"dest_port", (char *)"robot_model",
    (char *)"robot_model_name", (char *)"lidar_model",
    (char *)"lidar_scan_freq_hz", (char *)"motor_model",
    (char *)"base_dia", (char *)"wheel_track",
    (char *)"base_wheel_dia", (char *)"max_wheel_accel",
    (char *)"motor_max_rpm", (char *)"wheel_ppr", (char *)"motor_voltage",
    (char *)"motor_driver_type", (char *)"motor_encoder_type",
    (char *)"motor_direction_reversed", (char *)"motor_encoder_reversed",
    (char *)"motor_pid_kp", (char *)"motor_pid_ki",
    (char *)"motor_pid_kd", (char *)"motor_pid_mode",
    (char *)"motor_pid_period", (char *)"motor_max_rpm_derate",
    (char *)"battery_adc_attenuation",};

public:
  static const uint8_t ERR_REBOOT_BLINK_CYCLES = 3; // Blink out an error a few times, then reboot
  static const uint32_t LONG_BLINK_MS = 1000;
  static const uint32_t LONG_BLINK_PAUSE_MS = 2000;
  static const uint32_t SHORT_BLINK_MS = 200;
  static const uint32_t SHORT_BLINK_PAUSE_MS = 500;
  static const uint32_t SPIN_TELEM_STATS = 100;

  // Micro-ROS config
  static constexpr char * UROS_NODE_NAME = (char *)"pet"; // temp hardcoded
  static constexpr char * UROS_TELEM_TOPIC_NAME = (char *)"telemetry";
  static constexpr char * UROS_LOG_TOPIC_NAME = (char *)"rosout";
  //static constexpr char * UROS_DIAG_TOPIC_NAME = (char *)"diagnostics";
  static constexpr char * UROS_CMD_VEL_TOPIC_NAME = (char *)"cmd_vel";
  static const uint32_t UROS_PING_PUB_PERIOD_US = 10*1000*1000;
  static const uint32_t UROS_TELEM_PUB_PERIOD_US = 50*1000;
  static const uint32_t UROS_TIME_SYNC_TIMEOUT_MS = 1000;
  static const uint32_t UROS_PARAMS_UPDATE_PERIOD_US = 500*1000;

  // ROS Parameters
  const uint8_t UROS_PARAM_COUNT = 25;
  static constexpr char * UROS_PARAM_LIDAR_SCAN_FREQ_TARGET = (char *)"lidar.scan.freq.target";
  static constexpr char * UROS_PARAM_LIDAR_SCAN_FREQ_NOW = (char *)"lidar.scan.freq.now";
  static constexpr char * UROS_PARAM_MOTOR_LEFT_ENCODER_NOW = (char *)"motor.left.encoder.now";
  static constexpr char * UROS_PARAM_MOTOR_RIGHT_ENCODER_NOW = (char *)"motor.right.encoder.now";

  static constexpr char * UROS_PARAM_MOTOR_LEFT_RPM_NOW = (char *)"motor.left.rpm.now";
  static constexpr char * UROS_PARAM_MOTOR_LEFT_RPM_TARGET = (char *)"motor.left.rpm.target";
  static constexpr char * UROS_PARAM_MOTOR_LEFT_PWM_NOW = (char *)"motor.left.pwm.now";

  static constexpr char * UROS_PARAM_MOTOR_RIGHT_RPM_NOW = (char *)"motor.right.rpm.now";
  static constexpr char * UROS_PARAM_MOTOR_RIGHT_RPM_TARGET = (char *)"motor.right.rpm.target";
  static constexpr char * UROS_PARAM_MOTOR_RIGHT_PWM_NOW = (char *)"motor.right.pwm.now";

  static constexpr char * UROS_PARAM_MOTOR_ENCODER_PPR = (char *)"motor.encoder.ppr";
  static constexpr char * UROS_PARAM_MOTOR_ENCODER_TPR = (char *)"motor.encoder.tpr";
  static constexpr char * UROS_PARAM_MOTOR_RPM_MAX_DERATED = (char *)"motor.rpm.max.derated";
  static constexpr char * UROS_PARAM_MOTOR_PID_KP = (char *)"motor.pid.kp";
  static constexpr char * UROS_PARAM_MOTOR_PID_KI = (char *)"motor.pid.ki";
  static constexpr char * UROS_PARAM_MOTOR_PID_KD = (char *)"motor.pid.kd";
  static constexpr char * UROS_PARAM_MOTOR_PID_ON_ERROR = (char *)"motor.pid.on_error";
  static constexpr char * UROS_PARAM_MOTOR_PID_PERIOD = (char *)"motor.pid.period";

  static constexpr char * UROS_PARAM_MAX_WHEEL_ACCEL = (char *)"base.wheel.accel.max";
  static constexpr char * UROS_PARAM_BASE_DIA = (char *)"base.diameter";
  static constexpr char * UROS_PARAM_BASE_WHEEL_TRACK = (char *)"base.wheel.track";
  static constexpr char * UROS_PARAM_BASE_WHEEL_DIA = (char *)"base.wheel.diameter";

  static const uint16_t LIDAR_BUF_LEN = 400;
  static const uint16_t LIDAR_SERIAL_RX_BUF_LEN = 1024;

  static const uint32_t WIFI_CONN_TIMEOUT_MS = 30000;
  static constexpr char * PARAM_AP_WIFI_SSID = (char *) "MAKER'S PET";

public:
  // Hack  
  // Cache divisions
  float speed_diff_to_us;
  float base_wheel_accel_max;
  float wheel_track_recip;
  float wheel_track_radius;
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
  void setWheelDia(float wheel_dia) {
    wheel_radius = wheel_dia * 0.5;
    wheel_perim_len_div60 = PI * wheel_dia / 60;
    wheel_perim_len_div60_recip = 1/wheel_perim_len_div60;
  }
  
  void setMaxWheelAccel(float max_wheel_accel) {
    base_wheel_accel_max = max_wheel_accel;
    speed_diff_to_us = 1e6/max_wheel_accel;
  }
  
  void setWheelTrack(float wheel_track) {
    wheel_track_radius = wheel_track*0.5f;
    wheel_track_recip = 1/wheel_track;
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
    float ang_component = speed_ang_z*wheel_track_radius;
    *speed_right = speed_lin_x + ang_component;
    *speed_left  = speed_lin_x - ang_component;
  }
};
