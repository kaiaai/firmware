// Copyright 2023-2025 KAIA.AI
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
  static constexpr char* FW_VERSION = (char*)"0.8.3-iron";
  static constexpr char* CONFIG_YAML_PATH = (char *)"/config.yaml";
  static constexpr char* NETWORK_YAML_PATH = (char *)"/network.yaml";
  static constexpr char* INDEX_HTML_PATH = (char *)"/index.html";
  
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

public:
  static const uint8_t ERR_REBOOT_BLINK_CYCLES = 3; // Blink out an error a few times, then reboot
  static const uint32_t LONG_BLINK_MS = 1000;
  static const uint32_t LONG_BLINK_PAUSE_MS = 2000;
  static const uint32_t SHORT_BLINK_MS = 200;
  static const uint32_t SHORT_BLINK_PAUSE_MS = 500;
  static const uint32_t SPIN_TELEM_STATS = 100;
  static const uint16_t LIDAR_BUF_LEN = 400;
  static const uint16_t LIDAR_SERIAL_RX_BUF_LEN = 1024;
  static const uint32_t WIFI_CONN_TIMEOUT_MS = 30000;
  static constexpr char * SSID_AP = (char *) "KAIA.AI";
  static const uint32_t MONITOR_BAUD = 115200;
  static const uint8_t UNDEFINED_GPIO = 255;

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
  const uint8_t UROS_PARAM_COUNT = 20;
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
  //static constexpr char * UROS_PARAM_MOTOR_ENCODER_TPR = (char *)"motor.encoder.tpr";
  static constexpr char * UROS_PARAM_MOTOR_RPM_MAX = (char *)"motor.rpm.max";
  static constexpr char * UROS_PARAM_MOTOR_PID_KP = (char *)"motor.driver.pid.kp";
  static constexpr char * UROS_PARAM_MOTOR_PID_KI = (char *)"motor.driver.pid.ki";
  static constexpr char * UROS_PARAM_MOTOR_PID_KD = (char *)"motor.driver.pid.kd";
  static constexpr char * UROS_PARAM_MOTOR_PID_KPM = (char *)"motor.driver.pid.kpm";
  static constexpr char * UROS_PARAM_MOTOR_PID_PERIOD = (char *)"motor.driver.pid.period";

  static constexpr char * UROS_PARAM_BASE_WHEEL_ACCEL_MAX = (char *)"base.wheel.accel.max";
  //static constexpr char * UROS_PARAM_BASE_DIA = (char *)"base.diameter";
  static constexpr char * UROS_PARAM_BASE_WHEEL_TRACK = (char *)"base.wheel.track";
  static constexpr char * UROS_PARAM_BASE_WHEEL_DIA = (char *)"base.wheel.diameter";

public:
  String ssid = "";
  String pass = "";
  String dest_ip = "";
  String board_manufacturer = "N/A";
  String board_model = "N/A";
  String board_version = "N/A";
  unsigned int dest_port = 8888;
  float base_wheel_dia = 0.043f;
  float base_wheel_accel_max = 1.0;
  float base_wheel_track = 0.105043f;
  uint8_t led_sys_gpio = UNDEFINED_GPIO;
  uint8_t led_sys_invert = false;
  uint8_t button_boot_gpio = UNDEFINED_GPIO;
  uint8_t button_boot_invert = true;
  uint8_t monitor_gpio_tx = UNDEFINED_GPIO;
  String motor_driver_type = "IN1_IN2";
  String motor_encoder_type = "AB_QUAD";
  float motor_rpm_max = 200;
  float motor_encoder_ppr = 1035;
  float motor_driver_pid_period = 0.03f;
  float motor_driver_pid_kp = 0.001f;
  float motor_driver_pid_ki = 0.003f;
  float motor_driver_pid_kd = 0;
  float motor_driver_pid_kpm = 0;
  bool mot_left_drv_reverse = false;
  bool mot_right_drv_reverse = false;
  bool mot_left_enc_reverse = false;
  bool mot_right_enc_reverse = false;
  //float base_diameter = 0.127f;
  //String base_model = "makerspet_mini";
  String lidar_model = "LDROBOT LD14P";
  float adc_bat_atten = 3.7f;
  float adc_bat_voltage_full = 9.6f;
  float adc_bat_voltage_empty = 7.2f;
  uint8_t adc_bat_gpio = 255;
  uint8_t lidar_gpio_pwm = 255;
  uint8_t lidar_gpio_en = 255;
  uint8_t lidar_gpio_rx = 255;
  uint8_t lidar_gpio_tx = 255;
  float lidar_scan_freq_target = 0;
  uint8_t mot_left_enc_gpio_a_fg = 255;
  uint8_t mot_left_enc_gpio_b = 255;
  uint8_t mot_right_enc_gpio_a_fg = 255;
  uint8_t mot_right_enc_gpio_b = 255;
  uint8_t mot_right_drv_gpio_in1 = 255;
  uint8_t mot_left_drv_gpio_in1_pwm = 255;
  uint8_t mot_left_drv_gpio_in2_cw = 255;
  uint8_t mot_right_drv_gpio_in1_pwm = 255;
  uint8_t mot_right_drv_gpio_in2_cw = 255;
  uint32_t monitor_baud = 115200;

  // Cache divisions
  float speed_diff_to_us;
  float base_wheel_track_recip = 1.0f / base_wheel_track;
  float wheel_radius;
  float wheel_perim_len_div60;
  float wheel_perim_len_div60_recip;

public:

  String load(const char* file_path) {
    File file = SPIFFS.open(file_path);
    if (!file || file.isDirectory())
      return String(file_path) + " not found";

    int line = 0;
    String s, field_name;
    const uint8_t MAX_LEVEL = 10;
    uint32_t level_ident[MAX_LEVEL] = {0};
    String level_name[MAX_LEVEL];
    uint8_t levels = 0;

    while (file.available()) {
      s = file.readStringUntil('\n');
      line++;
      String ws = s;
      ws.trim();
      if (s.startsWith("#") || (ws.length() == 0))
        continue;

      if (s.length() > 80)
        return "Line too long";

      int colon_idx = s.indexOf(':');
      //Serial.println("colon_idx=" + String(colon_idx));
      if (colon_idx < 0) {
        //Serial.println("colon_idx<0");
        return "Missing colon in line " + String(line);
      }

      uint32_t ident;
      for(ident = 0; ident < s.length(); ident++) {
        if (s[ident] != ' ')
          break;
      }

      //Serial.println("ident=" + String(ident));
      if (ident == colon_idx) {
        //Serial.println("ident==colon_idx");
        return "Missing field name in line " + String(line);
      }

      field_name = s.substring(ident, colon_idx);
      //Serial.println("field_name=" + field_name);
      // check field name valid

      // find level
      uint8_t level;
      uint32_t id = 0;
      //Serial.println("levels=" + String(levels));
      if (levels > 0) {
        for(level = 0; level < levels; level++) {
          id = level_ident[level];
          if (ident <= id)
            break;
        }
        //Serial.println("id=" + String(id) + ", level=" + String(level));

        if (ident > id) {
          levels++;
          //Serial.println("ident > id");
          if (levels >= MAX_LEVEL) {
            //Serial.println("levels >= MAX_LEVEL");
            return "Max nesting level exceeded in line " + String(line);
          }
        } else
          levels = level+1;
      } else {
        levels = 1;
        //Serial.println("levels==0 -> levels:=1");
      }

      //Serial.println("levels=" + String(levels));
      level_ident[levels-1] = ident;
      level_name[levels-1] = field_name;

      // zero out upper levels
      //Serial.println("level_ident[" + String(levels) + "]:=0");
      level_ident[levels] = 0;

      if (s.length() > colon_idx + 2) {
        //Serial.println("s.length() " + String(s.length()) + " > colon_idx+2");
        if (s[colon_idx+1] != ' ') {
          //Serial.println("s[colon_idx+1] " + String(s[colon_idx+1]) + " != space");
          return "Space after colon expected in line " + String(line);
        }
        String param_value = s.substring(colon_idx+2);
        //Serial.println("param_value=" + param_value);
        set_param(level_name, param_value, levels);
      }
    }

    return "";
  }

  void set_param(String* lname, const String & pvalue, uint8_t nlevels) {

    //for (uint8_t level = 0; level < nlevels; level++) {
    //  Serial.print(lname[level]);    
    //  Serial.print((level < nlevels-1) ? '.' : '=');
    //}
    //Serial.println(pvalue);

    // TODO check for errors, return error as string
    if (nlevels == 1) {
      if (lname[0] == "ssid")
        ssid = trimString(pvalue);
      else if (lname[0] == "pass")
        pass = trimString(pvalue);
      else if (lname[0] == "dest_ip")
        dest_ip = trimString(pvalue);
      else if (lname[0] == "dest_port")
        dest_port = (unsigned int) pvalue.toInt();
      return;
    }

    if (nlevels == 2 && lname[0] == "board") {
      if (lname[1] == "manufacturer")
        board_manufacturer = trimString(pvalue);
      else if (lname[1] == "model")
        board_model = trimString(pvalue);
      else if (lname[1] == "version")
        board_version = trimString(pvalue);
      return;
    }

    if (lname[0] == "lidar") {
      switch(nlevels) {
        case 2:
          if (lname[1] == "model")
            lidar_model = trimString(pvalue);
          break;
        case 3:
          if (lname[1] == "gpio") {
            if (lname[2] == "tx")
              lidar_gpio_tx = (uint8_t) pvalue.toInt();
            else if (lname[2] == "rx")
              lidar_gpio_rx = (uint8_t) pvalue.toInt();
            else if (lname[2] == "pwm")
              lidar_gpio_pwm = (uint8_t) pvalue.toInt();
            else if (lname[2] == "en")
              lidar_gpio_en = (uint8_t) pvalue.toInt();
          }
          break;
        case 4:
          if (lname[1] == "scan" && lname[2] == "freq" && lname[3] == "target")
            lidar_scan_freq_target = pvalue.toFloat();
          break;
        default:
          break;
      }
      return;
    }

    if (lname[0] == "monitor") {
      switch(nlevels) {
        case 2:
          if (lname[1] == "baud")
            monitor_baud = (uint32_t) pvalue.toInt();
          break;
        case 3:
          if (lname[1] == "gpio" && lname[2] == "tx")
            monitor_gpio_tx = (uint8_t) pvalue.toInt();
          break;
      }
      return;
    }

    if (nlevels >= 3 && lname[0] == "base") {
      if (lname[1] == "wheel") {
        if (nlevels == 3) {
          if (lname[2] == "track")
            setWheelTrack(pvalue.toFloat());
          else if (lname[2] == "diameter")
            setWheelDia(pvalue.toFloat());
        } else if (nlevels == 4 && lname[2] == "accel" && lname[3] == "max")
          setMaxWheelAccel(pvalue.toFloat());
      }
      return;
    }

    if (nlevels == 4 && lname[0] == "led") {
      if (lname[1] == "system" && lname[2] == "driver") {
        if (lname[3] == "gpio")
          led_sys_gpio = (uint8_t) pvalue.toInt();
        else if (lname[3] == "invert")
          led_sys_invert = stringToBool(pvalue);
      }
      return;
    }

    if (nlevels == 3 && lname[0] == "button") {
      if (lname[1] == "boot") {
        if (lname[2] == "gpio")
          button_boot_gpio = (uint8_t) pvalue.toInt();
        else if (lname[2] == "invert")
          button_boot_invert = stringToBool(pvalue);
      }
      return;
    }

    if (lname[0] == "motor") {
      if (nlevels == 3 && lname[1] == "rpm" && lname[2] == "max")
        motor_rpm_max = pvalue.toFloat();
      else if (nlevels >= 3 && lname[1] == "driver") {
        if (nlevels == 3 && lname[2] == "type")
          motor_driver_type = trimString(pvalue);
        else if (nlevels == 4 && lname[2] == "pid") {
          if (lname[3] == "period")
            motor_driver_pid_period = pvalue.toFloat();
          else if (lname[3] == "kp")
            motor_driver_pid_kp = pvalue.toFloat();
          else if (lname[3] == "ki")
            motor_driver_pid_ki = pvalue.toFloat();
          else if (lname[3] == "kd")
            motor_driver_pid_kd = pvalue.toFloat();
          else if (lname[3] == "kpm")
            motor_driver_pid_kpm = pvalue.toFloat();
        }
      } else if (nlevels == 3 && lname[1] == "encoder") {
        if (lname[2] == "type")
          motor_encoder_type = trimString(pvalue);
        else if (lname[2] == "ppr")
          motor_encoder_ppr = pvalue.toFloat();
      } else {
        bool left = lname[1] == "left";
        if (nlevels >= 4 && (left || lname[1] == "right")) {
          if (lname[2] == "encoder") {
            if (nlevels == 4 && lname[3] == "reverse") {
              bool val = stringToBool(pvalue);
              if (left)
                mot_left_enc_reverse = val;
              else
                mot_right_enc_reverse = val;
            } else if (nlevels == 5 && (lname[3] == "gpio")) {
              if (lname[4] == "a" || lname[3] == "fg") {
                uint8_t gpio = (uint8_t) pvalue.toInt();
                if (left)
                  mot_left_enc_gpio_a_fg = gpio;
                else
                  mot_right_enc_gpio_a_fg = gpio;
              } else if (lname[4] == "b") {
                uint8_t gpio = (uint8_t) pvalue.toInt();
                if (left)
                  mot_left_enc_gpio_b = gpio;
                else
                  mot_right_enc_gpio_b = gpio;
              }
            }
          } else if (lname[2] == "driver") {
            if (nlevels == 4 && lname[3] == "reverse") {
              bool val = stringToBool(pvalue);
              if (left)
                mot_left_drv_reverse = val;
              else
                mot_right_drv_reverse = val;
            } else if (nlevels == 5 && (lname[3] == "gpio")) {
              if (lname[4] == "in1" || lname[4] == "pwm") {
                uint8_t gpio = (uint8_t) pvalue.toInt();
                if (left)
                  mot_left_drv_gpio_in1_pwm = gpio;
                else
                  mot_right_drv_gpio_in1_pwm = gpio;
              } else if (lname[4] == "in2" || lname[4] == "cw") {
                uint8_t gpio = (uint8_t) pvalue.toInt();
                if (left)
                  mot_left_drv_gpio_in2_cw = gpio;
                else
                  mot_right_drv_gpio_in2_cw = gpio;
              }
            }
          }
        }
      }
      return;
    }

    if (nlevels >= 3 && lname[0] == "adc" && lname[1] == "battery") {
      if (nlevels == 3) {
        if (lname[2] == "attenuation")
          adc_bat_atten = pvalue.toFloat();
        else if (lname[2] == "gpio")
          adc_bat_gpio = (uint8_t) pvalue.toInt();
      } else if (nlevels == 4 && lname[2] == "voltage") {
        if (lname[3] == "full")
          adc_bat_voltage_full = pvalue.toFloat();
        if (lname[3] == "empty")
          adc_bat_voltage_empty = pvalue.toFloat();
      }
      return;
    }
  }

  // Hack
  void setWheelDia(float wheel_dia) {
    base_wheel_dia = wheel_dia;
    wheel_radius = wheel_dia * 0.5;
    wheel_perim_len_div60 = PI * wheel_dia / 60;
    wheel_perim_len_div60_recip = 1/wheel_perim_len_div60;
  }
  
  void setMaxWheelAccel(float max_wheel_accel) {
    base_wheel_accel_max = max_wheel_accel;
    speed_diff_to_us = 1e6/max_wheel_accel;
  }
  
  void setWheelTrack(float wheel_track) {
    base_wheel_track = wheel_track;
    base_wheel_track_recip = 1/wheel_track;
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
    float ang_component = speed_ang_z*base_wheel_track*0.5f;
    *speed_right = speed_lin_x + ang_component;
    *speed_left  = speed_lin_x - ang_component;
  }

  String trimString(String s) {
    s.trim();
    return s;
  }
  
  bool stringToBool(String s) {
    s.trim();
    s.toLowerCase();
    return s == "true";
  }
};
