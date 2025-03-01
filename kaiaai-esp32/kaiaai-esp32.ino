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

#ifndef ESP32
  #error This code runs on ESP32
#endif

#include "robot_config.h"
#include "util.h"
#include <WiFi.h>
#include <stdio.h>
#include "motors.h"
#include "ap.h"
#include "lidar.h"
#include "ros.h"
#include "adc.h"
#include <SPIFFS.h>

CONFIG cfg;
kaiaai_msgs__msg__JointPosVel joint[MOTOR_COUNT];
float joint_prev_pos[MOTOR_COUNT] = {0};
uint8_t lidar_buf[cfg.LIDAR_BUF_LEN] = {0};

unsigned long telem_prev_pub_time_us = 0;
unsigned long ping_prev_pub_time_us = 0;
unsigned long ros_params_update_prev_time_us = 0;

unsigned long ramp_duration_us = 0;
unsigned long ramp_start_time_us = 0;
float ramp_start_rpm_right = 0;
float ramp_start_rpm_left = 0;
float ramp_target_rpm_right = 0;
float ramp_target_rpm_left = 0;
bool ramp_enabled = true;

unsigned long stat_sum_spin_telem_period_us = 0;
unsigned long stat_max_spin_telem_period_us = 0;

#if ESP_IDF_VERSION_MAJOR >= 5
  #error Espressif IDF v5 is not yet supported
#endif

void twist_sub_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float target_speed_lin_x = msg->linear.x;
  float target_speed_ang_z = msg->angular.z;
  //Serial.print("linear.x ");
  //Serial.print(msg->linear.x);
  //Serial.print(", angular.z ");
  //Serial.println(msg->angular.z);

  if (msg->linear.y != 0) {
    Serial.print("Warning: /cmd_vel linear.y = ");
    Serial.print(msg->linear.y);
    Serial.println(" not zero");
  }

  // Twist to target wheel speeds
  float twist_target_speed_right = 0;
  float twist_target_speed_left = 0;

  cfg.twistToWheelSpeeds(target_speed_lin_x, target_speed_ang_z,
    &twist_target_speed_right, &twist_target_speed_left);

  twist_target_speed_left = -twist_target_speed_left;

  // Wheel speeds to RPM
  float twist_target_rpm_right = cfg.speed_to_rpm(twist_target_speed_right);
  float twist_target_rpm_left = cfg.speed_to_rpm(twist_target_speed_left);

  //Serial.print(", twist_target_rpm_right ");
  //Serial.print(twist_target_rpm_right);
  //Serial.print(", twist_target_rpm_left ");
  //Serial.print(twist_target_rpm_left);

  // Limit target RPM
  float limited_target_rpm_right =
    absMin(twist_target_rpm_right, motorRight.getMaxRPM());
  float limited_target_rpm_left =
    absMin(twist_target_rpm_left, motorLeft.getMaxRPM());

  //Serial.print(", limited_target_rpm_right ");
  //Serial.print(limited_target_rpm_right);
  //Serial.print(", limited_target_rpm_left ");
  //Serial.print(limited_target_rpm_left);

  // Scale down both target RPMs to within limits
  if (twist_target_rpm_right != limited_target_rpm_right ||
    twist_target_rpm_left != limited_target_rpm_left) {

    float rpm_scale_down_factor_right = 1;
    float rpm_scale_down_factor_left = 1;

    if (twist_target_rpm_right != 0) {
      rpm_scale_down_factor_right = limited_target_rpm_right /
        twist_target_rpm_right;
    }
    if (twist_target_rpm_left != 0) {
      rpm_scale_down_factor_left = limited_target_rpm_left /
        twist_target_rpm_left;
    }

    float rpm_scale_down_factor = min(rpm_scale_down_factor_right,
      rpm_scale_down_factor_left);   

    ramp_target_rpm_right = twist_target_rpm_right * rpm_scale_down_factor;
    ramp_target_rpm_left = twist_target_rpm_left * rpm_scale_down_factor;
  } else {
    ramp_target_rpm_right = twist_target_rpm_right;
    ramp_target_rpm_left = twist_target_rpm_left;
  }

  //Serial.print(", ramp_target_rpm_ri  ght ");
  //Serial.print(ramp_target_rpm_right);
  //Serial.print(", ramp_target_rpm_left ");
  //Serial.println(ramp_target_rpm_left);

  if (!ramp_enabled) {
    setMotorSpeeds(ramp_target_rpm_left, ramp_target_rpm_right);
    return;
  }

  // Calculate change in speeds
  ramp_start_rpm_right = motorRight.getTargetRPM();
  ramp_start_rpm_left = motorLeft.getTargetRPM();
  
  float ramp_start_speed_right = cfg.rpm_to_speed(ramp_start_rpm_right);
  float ramp_start_speed_left = cfg.rpm_to_speed(ramp_start_rpm_left);

  float ramp_target_speed_right = cfg.rpm_to_speed(ramp_target_rpm_right);
  float ramp_target_speed_left = cfg.rpm_to_speed(ramp_target_rpm_left);

  float ramp_speed_diff_right = ramp_target_speed_right - ramp_start_speed_right;
  float ramp_speed_diff_left = ramp_target_speed_left - ramp_start_speed_left;

  // Calculate time to accelerate
  float abs_speed_diff_right = abs(ramp_speed_diff_right);
  float abs_speed_diff_left = abs(ramp_speed_diff_left);
  float max_abs_speed_diff = max(abs_speed_diff_right, abs_speed_diff_left);

  ramp_duration_us = max_abs_speed_diff * cfg.speed_diff_to_us;
  ramp_start_time_us = esp_timer_get_time(); // Start speed ramp

  updateSpeedRamp();
}

void updateSpeedRamp() {
  if (ramp_target_rpm_right == motorRight.getTargetRPM() &&
    ramp_target_rpm_left == motorLeft.getTargetRPM()) {
    return;
  }

  unsigned long time_now_us = esp_timer_get_time();
  unsigned long ramp_elapsed_time_us = time_now_us - ramp_start_time_us;

  float rpm_right;
  float rpm_left;

  if (ramp_elapsed_time_us < ramp_duration_us) {
    float ratio = (float)ramp_elapsed_time_us / (float)ramp_duration_us; // 0..1
    float rpm_change_right = (ramp_target_rpm_right - ramp_start_rpm_right) * ratio;
    float rpm_change_left = (ramp_target_rpm_left - ramp_start_rpm_left) * ratio;

    rpm_right = ramp_start_rpm_right + rpm_change_right;
    rpm_left = ramp_start_rpm_left + rpm_change_left;
  } else {
    rpm_right = ramp_target_rpm_right;
    rpm_left = ramp_target_rpm_left;
  }

  setMotorSpeeds(rpm_left, rpm_right);
}

String set_param_callback(const char * param_name, const char * param_value) {

  static String text;

  if (param_name == NULL) {
    write_file(cfg.NETWORK_YAML_PATH, text.c_str());
    Serial.println(", restarting...");
    delay(100);
    ESP.restart();
    return "";
  } else {
    text = text + String(param_name) + ": " + String(param_value) + '\n';
    return strcmp(param_name, "pass") == 0 ? "****" : String(param_value);
  }
}

static inline bool initWiFi(const String & ssid, const String & passw) {

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passw);

  const uint32_t blink_delay = 500;
  unsigned long startMillis = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.print("Connecting to WiFi ");
    Serial.print(ssid);
    Serial.print(" ...");

    if (millis() - startMillis >= cfg.WIFI_CONN_TIMEOUT_MS) {
      Serial.println(" timed out");
      return false;
    }

    digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);
    delay(blink_delay);
    digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
    //Serial.print('.'); // F('.') crashes
    delay(blink_delay);
  }

  digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
  Serial.print(" connected, ");
  Serial.print("IP ");
  Serial.println(WiFi.localIP());
  //printWiFiChannel();
  return true;
}

void spinTelem(bool force_pub) {
  static int telem_pub_count = 0;
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - telem_prev_pub_time_us;

  if (!force_pub && (step_time_us < cfg.UROS_TELEM_PUB_PERIOD_US))
    return;

  publishTelem(step_time_us);
  telem_prev_pub_time_us = time_now_us;

  digiWrite(cfg.led_sys_gpio, !digiRead(cfg.led_sys_gpio, cfg.led_sys_invert),
    cfg.led_sys_invert);
  //if (++telem_pub_count % 5 == 0) {
  //  Serial.print("RPM L ");
  //  Serial.print(motorLeft.getCurrentRPM());
  //  Serial.print(" R ");
  //  Serial.println(motorRight.getCurrentRPM());
  //}

  stat_sum_spin_telem_period_us += step_time_us;
  stat_max_spin_telem_period_us = stat_max_spin_telem_period_us <= step_time_us ?
    step_time_us : stat_max_spin_telem_period_us;
  
  // How often telemetry gets published
  if (++telem_pub_count % cfg.SPIN_TELEM_STATS == 0) {
    String s = "Telem avg ";
    s = s + String(stat_sum_spin_telem_period_us / (1000*cfg.SPIN_TELEM_STATS));
    s = s + " max ";
    s = s + String(stat_max_spin_telem_period_us / 1000);
    s = s + "ms";

    float rpm = lidar->getCurrentScanFreqHz();
    if (rpm >= 0) {
      s = s + ", LiDAR RPM ";
      s = s + String(rpm);
    }

    s = s + ", wheels RPM ";
    s = s + String(motorLeft.getCurrentRPM()) + " ";
    s = s + String(motorRight.getCurrentRPM());

    s = s + ", battery " + String(telem_msg.battery_mv*0.001f) + "V";
    s = s + ", RSSI " + String(telem_msg.wifi_rssi_dbm) + "dBm";
    printlnNB(s);

    stat_sum_spin_telem_period_us = 0;
    stat_max_spin_telem_period_us = 0;
  }
}

void publishTelem(unsigned long step_time_us) {
  struct timespec tv = {0, 0};
  clock_gettime(CLOCK_REALTIME, &tv);
  telem_msg.stamp.sec = tv.tv_sec;
  telem_msg.stamp.nanosec = tv.tv_nsec;

  float joint_pos_delta[MOTOR_COUNT];
  float step_time = 1e-6 * (float)step_time_us;

  long rssi_dbm = WiFi.RSSI();
  rssi_dbm = rssi_dbm > 127 ? 127 : rssi_dbm;
  rssi_dbm = rssi_dbm < -128 ? -128 : rssi_dbm;
  telem_msg.wifi_rssi_dbm = (int8_t) rssi_dbm;

  float batt_mv = getBatteryMilliVolts();
  telem_msg.battery_mv = (uint16_t) round(batt_mv);

  //Serial.print(rssi_dbm);
  //Serial.print("dbm, ");
  //Serial.print(voltage_mv);
  //Serial.println("mV");

  for (unsigned char i = 0; i < MOTOR_COUNT; i++) {
    joint[i].pos = i == 0 ? motorLeft.getShaftAngle() : motorRight.getShaftAngle();
    joint_pos_delta[i] = joint[i].pos - joint_prev_pos[i];
    joint[i].vel = joint_pos_delta[i] / step_time;    
    joint_prev_pos[i] = joint[i].pos;
  }

  calcOdometry(step_time_us, joint_pos_delta[0], joint_pos_delta[1]);
//  calcOdometry2(step_time_us, joint_pos_delta[0], joint_pos_delta[1]);

  rcl_ret_t rc = rcl_publish(&telem_pub, &telem_msg, NULL);
  if (rc != RCL_RET_OK) {
    Serial.print("rcl_publish(telem_msg");
    Serial.print(") error ");
    Serial.println(rc);
  }
  
  //Serial.print(telem_msg.odom_pos_x, 8);
  //Serial.print("\t");
  //Serial.print(telem_msg.odom_pos_y, 8);
  //Serial.print("\t");
  //Serial.println(telem_msg.odom_pos_yaw, 8);
  
  telem_msg.lds.size = 0;
  telem_msg.seq++;
}

void calcOdometry(unsigned long step_time_us, float joint_pos_delta_right,
  float joint_pos_delta_left) {

  if (step_time_us == 0)
    return;

  float distance_right = -joint_pos_delta_right * cfg.wheel_radius;
  float distance_left = joint_pos_delta_left * cfg.wheel_radius;

  // TODO use Runge-Kutta integration for better accuracy
  float average_distance = (distance_right + distance_left) * 0.5;
  float d_yaw = (distance_left - distance_right)*cfg.base_wheel_track_recip;
//  if (abs(d_yaw) > 1) {
//    printNB("WARNING: odometry asin() domain out of bounds. Check motor encoders.");
//    d_yaw = d_yaw > 0 ? 1 : -1;
//  }
//  d_yaw = asin(d_yaw);

  // Average angle during the motion
  float average_angle = d_yaw*0.5 + telem_msg.odom_pos_yaw;

  if (average_angle > PI)
    average_angle -= TWO_PI;
  else if (average_angle < -PI)
    average_angle += TWO_PI;

  // Calculate the new pose (x, y, and theta)
  float d_x = cos(average_angle) * average_distance;
  float d_y = sin(average_angle) * average_distance;

  telem_msg.odom_pos_x += d_x;
  telem_msg.odom_pos_y += d_y;
  telem_msg.odom_pos_yaw += d_yaw;

  if (telem_msg.odom_pos_yaw > PI)
    telem_msg.odom_pos_yaw -= TWO_PI;
  else if (telem_msg.odom_pos_yaw < -PI)
    telem_msg.odom_pos_yaw += TWO_PI;

  float d_time = 1e-6 * (float)step_time_us;
  telem_msg.odom_vel_x = average_distance / d_time;
  telem_msg.odom_vel_yaw = d_yaw / d_time;
}

void spinPing() {
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - ping_prev_pub_time_us;
  
  if (step_time_us >= cfg.UROS_PING_PUB_PERIOD_US) {
    // timeout_ms, attempts
    rmw_uros_ping_agent(1, 1); //rmw_ret_t rc =
    ping_prev_pub_time_us = time_now_us;
    //Serial.println(rc == RCL_RET_OK ? "Ping OK" : "Ping error");
  }
}

void updateROSParams() {
  if (ros_config_params_changed) {
    ros_config_params_changed = false;
    rcl_ret_t ret = updateROSConfigParams();
    if (ret != RCL_RET_OK) {
      Serial.print("updateROSConfigParams() error ");
      Serial.println(ret);
    }
  }

  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - ros_params_update_prev_time_us;
  if (step_time_us >= cfg.UROS_PARAMS_UPDATE_PERIOD_US) {

    rcl_ret_t ret = updateROSRealTimeParams();
    if (ret != RCL_RET_OK) {
      Serial.print("updateROSRealTimeParams() error ");
      Serial.println(ret);
    }

    ros_params_update_prev_time_us = time_now_us;
  }
}

void loop() {
  static bool wifi_ok_prev = true;

  bool wifi_ok = WiFi.status() == WL_CONNECTED;
  if (wifi_ok && !wifi_ok_prev) {
    lidar->start();
    Serial.println("WiFi connection restored");
  } else if (!wifi_ok && wifi_ok_prev) {
    lidar->stop();
    Serial.println("WiFi connection lost: pausing motors, LiDAR");
  }
  wifi_ok_prev = wifi_ok;

  lidar->loop();

  // Process micro-ROS callbacks
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  if (ret != RCL_RET_OK) {
    Serial.print("rclc_executor_spin_some() error ");
    Serial.println(ret);
  }

  updateROSParams();

  spinTelem(false);
  spinPing();

  if (wifi_ok)
    updateSpeedRamp();
  else
    setMotorSpeeds(0, 0);

  motorLeft.update();
  motorRight.update();
}

bool isBootButtonPressed(uint8_t sec) {
  if (digiRead(cfg.button_boot_gpio, cfg.button_boot_invert))
    Serial.println("BOOT button pressed. Keep pressing for web config.");
  else
    return false;

  uint32_t msec = sec * 1000;
  unsigned long start_time_ms = millis();
  while (digiRead(cfg.button_boot_gpio, cfg.button_boot_invert)) {
    delay(50);
    digiWrite(cfg.led_sys_gpio, !digiRead(cfg.led_sys_gpio, cfg.led_sys_invert),
      cfg.led_sys_invert);
    if (millis() - start_time_ms > msec)
      return true;
  }
  return false;
}

void resetTelemMsg() {
  telem_msg.seq = 0;
  telem_msg.odom_pos_x = 0;
  telem_msg.odom_pos_y = 0;
  telem_msg.odom_pos_yaw = 0;
  telem_msg.odom_vel_x = 0;
  telem_msg.odom_vel_yaw = 0;
  
  telem_msg.joint.data = joint;
  telem_msg.joint.capacity = MOTOR_COUNT;
  telem_msg.joint.size = MOTOR_COUNT;

  telem_msg.lds.data = lidar_buf;
  telem_msg.lds.capacity = cfg.LIDAR_BUF_LEN;
  telem_msg.lds.size = 0;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    joint[i].pos = 0;
    joint[i].vel = 0;
    joint_prev_pos[i] = 0;
  }

  telem_msg.battery_mv = 0;
  telem_msg.wifi_rssi_dbm = 0;
}

/*
void blink_error_code(int n_blinks) {
  unsigned int i = 0;
  while(i++ < cfg.ERR_REBOOT_BLINK_CYCLES) {
    blink(cfg.LONG_BLINK_MS, 1);
    digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
    delay(cfg.SHORT_BLINK_PAUSE_MS);
    blink(cfg.SHORT_BLINK_MS, n_blinks);
    delay(cfg.LONG_BLINK_PAUSE_MS);
  }
}

void error_loop(int n_blinks){
  lidar->stop();

  //char buffer[40];
  //sprintf(buffer, "Blinking error %d", n_blinks);  
  //logMsg(buffer, rcl_interfaces__msg__Log__FATAL);
  Serial.print("Blinking LED ");
  Serial.print(n_blinks);
  Serial.println(" times...");

  blink_error_code(n_blinks);

  Serial.println("Rebooting...");
  Serial.flush();

  ESP.restart();
}
*/

void setup() {

  bool spiffs_ok = SPIFFS.begin(true);
//  blink_error_code(cfg.ERR_SPIFFS_INIT);
  bool html_exists = false;
  if (spiffs_ok)
    html_exists = SPIFFS.exists(cfg.INDEX_HTML_PATH);

  bool wifi_yaml_exists = SPIFFS.exists(cfg.NETWORK_YAML_PATH);
  String wifi_yaml_err;
  if (wifi_yaml_exists)
    wifi_yaml_err = cfg.load(cfg.NETWORK_YAML_PATH);

  bool config_yaml_exists = SPIFFS.exists(cfg.CONFIG_YAML_PATH);
  String config_yaml_err;
  if (config_yaml_exists)
    config_yaml_err = cfg.load(cfg.CONFIG_YAML_PATH);

  Serial.begin(cfg.MONITOR_BAUD);
  setPinDrive(cfg.monitor_gpio_tx);
  while(!Serial)
    delay(0);

  Serial.println();
  Serial.print("Kaia.ai firmware version ");
  Serial.println(cfg.FW_VERSION);

  Serial.print("ESP IDF version ");
  Serial.println(esp_get_idf_version());

  if (spiffs_ok) {
    Serial.println("SPIFFS mounted successfully");
    if (!html_exists) {
      Serial.println("Sketch data not found. Please upload sketch data.");
      idle();
    }
  } else {
    Serial.println("Error mounting SPIFFS");
    idle();
  }

  if (wifi_yaml_exists) {
    Serial.print(cfg.NETWORK_YAML_PATH);
    Serial.print(" found; ");
    if (wifi_yaml_err.length() != 0) {
      Serial.print("error parsing: ");
      Serial.println(wifi_yaml_err);
    } else
      Serial.println("loaded OK");
  }

  if (config_yaml_exists) {
    Serial.print(cfg.CONFIG_YAML_PATH);
    Serial.print(" found; ");
    if (config_yaml_err.length() != 0) {
      Serial.print("error parsing: ");
      Serial.println(config_yaml_err);
    } else
      Serial.println("loaded OK");
  }

  setPinMode(cfg.led_sys_gpio, OUTPUT);
  digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);

  setPinMode(cfg.button_boot_gpio, INPUT);

  bool launch_web_config = false;

  if (cfg.ssid.length() == 0) {
    Serial.println("WiFi SSID unknown");
    launch_web_config = true;
  }

  if (cfg.dest_ip.length() == 0) {
    Serial.println("dest_ip unknown");
    launch_web_config = true;
  }

  Serial.println("To enter web config push-and-release EN, "
    "then push-and-hold BOOT within 1 sec");
  delay(1000);
  launch_web_config |= isBootButtonPressed(cfg.RESET_SETTINGS_HOLD_SEC);

  if (launch_web_config) {
    digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);

    AP ap;
    ap.obtainConfig(cfg.SSID_AP, set_param_callback);
    return;
  }

  Serial.print("Board model ");
  Serial.print(cfg.board_model);
  Serial.print(", version ");
  Serial.print(cfg.board_version);
  Serial.print(", manufacturer ");
  Serial.println(cfg.board_manufacturer);
  cfg.board_manufacturer = ""; // free up a little memory
  cfg.board_model = "";
  cfg.board_version = "";

  setupLIDAR();
  setupADC();
  setupMotors();

  while(!initWiFi(cfg.ssid, cfg.pass));

  set_microros_wifi_transports(cfg.dest_ip.c_str(), cfg.dest_port);
  delay(2000);

  setupMicroROS(&twist_sub_callback);

  //pubDiagnostics();

  rcl_ret_t rc = addROSParams();
  if (rc != RCL_RET_OK) {
    Serial.print("addROSParams(");
    Serial.print(") error ");
    Serial.println(rc);
  }

  ros_config_params_changed = true;
  updateROSParams();
  Serial.println("Micro-ROS initialized");

  //Serial.print("Diagnostics pub ");
  //Serial.println(pubDiagnostics() ? "OK" : "FAILED");
//  pubDiagnostics();
  
  resetTelemMsg();
  
  startLIDAR();
    //blink_error_code(cfg.ERR_LIDAR_START);
    //error_loop(cfg.ERR_LIDAR_START);
}
