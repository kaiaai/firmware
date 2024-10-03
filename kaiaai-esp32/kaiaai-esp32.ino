// Copyright 2023-2024 KAIA.AI
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
#include "param_file.h"
#include "lidar.h"
#include "ros.h"
#include "adc.h"

#define RCCHECK(fn,E) { rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK)error_loop(E);}
#define BLCHECK(fn) { CONFIG::error_blink_count temp_cnt = fn; \
  if(temp_cnt != CONFIG::ERR_NONE)error_loop(temp_cnt);}

CONFIG cfg;
PARAM_FILE params(cfg.getParamNames(), cfg.getParamValues(), cfg.PARAM_COUNT);

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

bool set_param_callback(const char * param_name, const char * param_value) {
  if (param_name != NULL)
    return params.setByName(param_name, param_value);

  params.save();
  Serial.println("Parameters saved, restarting..");
  delay(100);
  ESP.restart();

  return false;
}

static inline bool initWiFi(String ssid, String passw) {

  if(ssid.length() == 0){
    Serial.println("Undefined SSID");
    return false;
  }

  WiFi.mode(WIFI_STA);
  //localIP.fromString(ip.c_str());
  //localGateway.fromString(gateway.c_str());

  //if (!WiFi.config(localIP, localGateway, subnet)){
  //  Serial.println("STA Failed to configure");
  //  return false;
  //}

  WiFi.begin(ssid, passw);

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

    digitalWrite(cfg.LED_PIN, HIGH);
    delay(500);
    digitalWrite(cfg.LED_PIN, LOW);
    //Serial.print('.'); // Don't use F('.'), it crashes code!!
    delay(500);
  }

  digitalWrite(cfg.LED_PIN, LOW);
  Serial.print(" connected, ");
  Serial.print("IP ");
  Serial.println(WiFi.localIP());
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

  digitalWrite(cfg.LED_PIN, !digitalRead(cfg.LED_PIN));
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

  RCSOFTCHECK(rcl_publish(&telem_pub, &telem_msg, NULL));

  
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
  float d_yaw = (distance_left - distance_right)*cfg.wheel_track_recip;
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
    BLCHECK(updateROSConfigParams());
  }

  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - ros_params_update_prev_time_us;
  if (step_time_us >= cfg.UROS_PARAMS_UPDATE_PERIOD_US) {
    BLCHECK(updateROSRealTimeParams());
    ros_params_update_prev_time_us = time_now_us;
  }
}

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    lidar->stop();
    setMotorSpeeds(0, 0);
    Serial.println("WiFi connection lost: stopping motors, LiDAR.");
    return;
  }

  lidar->loop();

  // Process micro-ROS callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)), cfg.ERR_UROS_SPIN);
  updateROSParams();

  spinTelem(false);
  spinPing();
  updateSpeedRamp();

  motorLeft.update();
  motorRight.update();

/*
  printNB(String(motorLeft.getEncoderValue()));
  printNB("\t");
  printNB(String(motorRight.getEncoderValue()));
  printNB("\t");

  printNB(String(motorLeft.getCurrentRPM()));
  printNB("\t");
  printNB(String(motorRight.getCurrentRPM()));
  printNB("\t");

  printNB(String(motorLeft.getCurrentPWM()));
  printNB("\t");
  printNB(String(motorRight.getCurrentPWM()));
  printNB("\t");

  printNB(String(motorLeft.getTargetRPM()));
  printNB("\t");
  printNB(String(motorRight.getTargetRPM()));  
  printlnNB();
*/
}

void resetParams() {
  Serial.println("** Restarting in web config mode **");
  params.purge();
  digitalWrite(cfg.LED_PIN, HIGH);
  Serial.flush();
  delay(5000);

  ESP.restart();
}

bool isBootButtonPressed(uint8_t sec) {
  if (!digitalRead(0))
    Serial.println("BOOT button pressed. Keep pressing for web config.");
  else
    return false;

  uint32_t msec = sec * 1000;
  unsigned long start_time_ms = millis();
  while (!digitalRead(0)) {
    delay(50);
    digitalWrite(cfg.LED_PIN, !digitalRead(cfg.LED_PIN));
    if (millis() - start_time_ms > msec) {
      return true;
    }
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

void blink_error_code(int n_blinks) {
  unsigned int i = 0;
  while(i++ < cfg.ERR_REBOOT_BLINK_CYCLES){
    blink(cfg.LONG_BLINK_MS, 1);
    digitalWrite(cfg.LED_PIN, LOW);
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

void setup() {
  Serial.begin(115200); // 500000

  pinMode(0, INPUT);
  pinMode(cfg.LED_PIN, OUTPUT);
  digitalWrite(cfg.LED_PIN, HIGH);

  Serial.println();
  Serial.println();
  Serial.print("Kaia.ai firmware version ");
  Serial.println(cfg.FW_VERSION);
  Serial.println("To enter web config push-and-release EN, "
    "then push-and-hold BOOT within 1 sec");

  delay(1000);
  if (isBootButtonPressed(cfg.RESET_SETTINGS_HOLD_SEC)) {
    params.init();
    resetParams();
  }

  if (!params.init())
    blink_error_code(cfg.ERR_SPIFFS_INIT);

//  if (!params.load() ||
//      !initWiFi(params.get(cfg.PARAM_SSID), params.get(cfg.PARAM_PASS)))
  bool ok = params.load();
  if (ok) {
    cfg.setWheelDia(params.getAsFloat(cfg.PARAM_BASE_WHEEL_DIA));  
    cfg.setMaxWheelAccel(params.getAsFloat(cfg.PARAM_MAX_WHEEL_ACCEL));  
    cfg.setWheelTrack(params.getAsFloat(cfg.PARAM_BASE_WHEEL_TRACK));
  
    setupLIDAR();
    setupADC();
    setupMotors();
  }

  if (!ok || !initWiFi(params.get(cfg.PARAM_SSID), params.get(cfg.PARAM_PASS)))
  {
    digitalWrite(cfg.LED_PIN, HIGH);
    if (ok)
      params.save(true);

    AP ap;
    ap.obtainConfig(cfg.PARAM_AP_WIFI_SSID, set_param_callback);
    return;
  }

  set_microros_wifi_transports(params.get(cfg.PARAM_DEST_IP),
    params.getAsInt(cfg.PARAM_DEST_PORT));

  delay(2000);

  BLCHECK(setupMicroROS(&twist_sub_callback));
  //pubDiagnostics();

  BLCHECK(addROSParams());
  ros_config_params_changed = true;
  updateROSParams();
  Serial.println("Micro-ROS initialized");

  //Serial.print("Diagnostics pub ");
  //Serial.println(pubDiagnostics() ? "OK" : "FAILED");
//  pubDiagnostics();
  
  resetTelemMsg();
  
  if (startLIDAR() != LDS::RESULT_OK)
    blink_error_code(cfg.ERR_LIDAR_START);
    //error_loop(cfg.ERR_LIDAR_START);
}
