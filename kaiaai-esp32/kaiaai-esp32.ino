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

// TODO make ROS2 properties  

#ifndef ESP32
  #error This code runs on ESP32
#endif

#include "robot_config.h"
#include "util.h"
#include <WiFi.h>
#include <stdio.h>
#include <micro_ros_kaia.h>
#include <HardwareSerial.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry2.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
//#include <rmw_microros/discovery.h>
#include <rclc_parameter/rclc_parameter.h>
#include "drive.h"
#include "ap.h"
#include "param_file.h"
#include "lds_all_models.h"

#if !defined(IS_MICRO_ROS_KAIA_MIN_VERSION) || !IS_MICRO_ROS_KAIA_MIN_VERSION(2,0,7,4)
#error "Please upgrade micro_ros_kaia library version"
#endif

#define RCCHECK(fn,E) { rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){error_loop((E));}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){Serial.println("RCSOFTCHECK failed");}}

CONFIG cfg;
PARAM_FILE params(cfg.getParamNames(), cfg.getParamValues(), cfg.PARAM_COUNT); // temp hack
DriveController drive;
LDS *lds;

rcl_publisher_t telem_pub;
rcl_publisher_t log_pub;
rcl_subscription_t twist_sub;
kaiaai_msgs__msg__KaiaaiTelemetry2 telem_msg;
geometry_msgs__msg__Twist twist_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_parameter_server_t param_server;

HardwareSerial LdSerial(2); // TX 17, RX 16

kaiaai_msgs__msg__JointPosVel joint[drive.MOTOR_COUNT];
float joint_prev_pos[drive.MOTOR_COUNT] = {0};
uint8_t lds_buf[cfg.LDS_BUF_LEN] = {0};

unsigned long telem_prev_pub_time_us = 0;
unsigned long ping_prev_pub_time_us = 0;
unsigned long telem_pub_period_us = cfg.UROS_TELEM_PUB_PERIOD_MS*1000;
unsigned long ping_pub_period_us = cfg.UROS_PING_PUB_PERIOD_MS*1000;

unsigned long ramp_duration_us = 0;
unsigned long ramp_start_time_us = 0;
float ramp_start_rpm_right = 0;
float ramp_start_rpm_left = 0;
float ramp_target_rpm_right = 0;
float ramp_target_rpm_left = 0;
bool ramp_enabled = true;

unsigned long stat_sum_spin_telem_period_us = 0;
unsigned long stat_max_spin_telem_period_us = 0;

size_t lds_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LdSerial.write(buffer, length);
}

int lds_serial_read_callback() {
/*
  static int i=0;

  int c = LdSerial.read();
  if (c < 0)
    return c;

  if (c < 16)
    Serial.print('0');
  Serial.print(c, HEX);
  if (i++ % 16 == 0)
    Serial.println();
  else
    Serial.print(' ');
  return c;
*/
  return LdSerial.read();
}

void twist_sub_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float target_speed_lin_x = msg->linear.x;
  float target_speed_ang_z = msg->angular.z;

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

  // Limit target RPM
  float limited_target_rpm_right =
    absMin(twist_target_rpm_right, drive.getMaxRPM());
  float limited_target_rpm_left =
    absMin(twist_target_rpm_left, drive.getMaxRPM());

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

  if (!ramp_enabled) {
    setMotorSpeeds(ramp_target_rpm_right, ramp_target_rpm_left);
    return;
  }

  // Calculate change in speeds
  ramp_start_rpm_right = drive.getTargetRPM(drive.MOTOR_RIGHT);
  ramp_start_rpm_left = drive.getTargetRPM(drive.MOTOR_LEFT);
  
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

void setMotorSpeeds(float ramp_target_rpm_right, float ramp_target_rpm_left) {
  drive.setRPM(drive.MOTOR_RIGHT, ramp_target_rpm_right);
  drive.setRPM(drive.MOTOR_LEFT, ramp_target_rpm_left);
}

void updateSpeedRamp() {
  if (ramp_target_rpm_right == drive.getTargetRPM(drive.MOTOR_RIGHT) &&
    ramp_target_rpm_left == drive.getTargetRPM(drive.MOTOR_LEFT)) {
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

  setMotorSpeeds(rpm_right, rpm_left);
}

void setup() {
  Serial.begin(115200);

  pinMode(cfg.LED_PIN, OUTPUT);
  digitalWrite(cfg.LED_PIN, HIGH);

  Serial.println();
  Serial.print("Kaia.ai firmware version ");
  Serial.println(cfg.FW_VERSION);

  delay(1000);
  if (isBootButtonPressed(cfg.RESET_SETTINGS_HOLD_MS)) {
    params.init();
    resetSettings();
  }

  if (!params.init())
    blink_error_code(cfg.ERR_SPIFFS_INIT);

  if (!params.load() ||
    !initWiFi(params.get(cfg.PARAM_SSID), params.get(cfg.PARAM_PASS))) {
    digitalWrite(cfg.LED_PIN, HIGH);

    AP ap;
    ap.obtainConfig(cfg.PARAM_AP_WIFI_SSID, set_param_callback);
    return;
  }

  setupADC();
  setupLDS();

  set_microros_wifi_transports(params.get(cfg.PARAM_DEST_IP),
    String(params.get(cfg.PARAM_DEST_PORT)).toInt());

  delay(2000);

  initRos();
  Serial.println("Micro-ROS initialized");
  
  if (startLDS() != LDS::RESULT_OK)
    blink_error_code(cfg.ERR_LDS_START);
    //error_loop(cfg.ERR_LDS_START);
  
  drive.init(cfg.MOT_PWM_LEFT_PIN, cfg.MOT_PWM_RIGHT_PIN,
    cfg.MOT_CW_LEFT_PIN, cfg.MOT_CW_RIGHT_PIN,
    cfg.MOT_FG_LEFT_PIN, cfg.MOT_FG_RIGHT_PIN);

  drive.resetEncoders();
  drive.setMaxRPM(String(params.get(cfg.PARAM_MOTOR_MAX_RPM)).toFloat()
    * cfg.MOTOR_MAX_RPM_DERATE);
  drive.setEncoderPPR(String(params.get(cfg.PARAM_WHEEL_PPR)).toFloat());

  cfg.setWheelDia(params.get(cfg.PARAM_WHEEL_DIA_MM));  
  cfg.setMaxWheelAccel(params.get(cfg.PARAM_MAX_WHEEL_ACCEL));  
  cfg.setWheelBase(params.get(cfg.PARAM_WHEEL_BASE_MM));
}

void setupADC() {

  if (!adcAttachPin(cfg.BAT_ADC_PIN))
    Serial.println("adcAttachPin() FAILED");

  unsigned int voltage_mv = analogReadMilliVolts(cfg.BAT_ADC_PIN);
  voltage_mv *= cfg.BAT_ADC_MULTIPLIER;
  if (voltage_mv < cfg.BAT_PRESENT_MV_MIN)
    voltage_mv = 0;

  Serial.print("Battery ");
  if (voltage_mv == 0) {
    Serial.println("NOT detected");
    Serial.println("Is the battery connected? Is the power switch on?");
  } else {
    Serial.print("voltage ");
    Serial.print(voltage_mv*0.001f);
    Serial.println("V");
  }
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

static inline void initRos() {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator), cfg.ERR_UROS_INIT);

  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  // https://github.com/micro-ROS/micro-ROS-demos/blob/iron/rclc/autodiscover_agent/main.c
  // https://micro.ros.org/blog/2020/09/30/discovery/
  // https://micro.ros.org/docs/api/rmw/
  // https://github.com/micro-ROS/rmw_microxrcedds/blob/iron/rmw_microxrcedds_c/include/rmw_microros/discovery.h
  // https://github.com/micro-ROS/rmw_microxrcedds/blob/iron/rmw_microxrcedds_c/src/rmw_microros/discovery.c
  // Auto discover micro-ROS agent
  // RMW_UXRCE_TRANSPORT=custom
  // RMW_UXRCE_TRANSPORT_UDP
  //Serial.print("micro-ROS agent ");
  //if (rmw_uros_discover_agent(rmw_options) == RCL_RET_OK) {
  //  Serial.println("not ");
  //}
  //Serial.print("found");

  RCCHECK(rmw_uros_options_set_client_key(cfg.UROS_CLIENT_KEY, rmw_options),
    cfg.ERR_UROS_INIT); // TODO multiple bots

  Serial.print(F("Connecting to Micro-ROS agent "));
  Serial.print(params.get(cfg.PARAM_DEST_IP));
  Serial.print(" ... ");

  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator), cfg.ERR_UROS_AGENT_CONN);
  //RCCHECK(rclc_support_init_with_options(&support, 0, NULL,
  //  &init_options, &allocator), cfg.ERR_UROS_AGENT_CONN);
  rcl_ret_t temp_rc = rclc_support_init_with_options(&support, 0, NULL,
    &init_options, &allocator);
  if (temp_rc != RCL_RET_OK) {
    Serial.println("failed");
    error_loop(cfg.ERR_UROS_AGENT_CONN);
  }
  Serial.println("success");

  syncRosTime();
  printCurrentTime();

  // https://micro.ros.org/docs/tutorials/programming_rcl_rclc/node/
  RCCHECK(rclc_node_init_default(&node, params.get(cfg.PARAM_ROBOT_MODEL_NAME),
    "", &support), cfg.ERR_UROS_NODE);

  RCCHECK(rclc_subscription_init_default(&twist_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    cfg.UROS_CMD_VEL_TOPIC_NAME), cfg.ERR_UROS_PUBSUB);

  RCCHECK(rclc_publisher_init_best_effort(&telem_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(kaiaai_msgs, msg, KaiaaiTelemetry2),
    cfg.UROS_TELEM_TOPIC_NAME), cfg.ERR_UROS_PUBSUB);

  RCCHECK(rclc_publisher_init_default(&log_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    cfg.UROS_LOG_TOPIC_NAME), cfg.ERR_UROS_PUBSUB);

  // https://github.com/ros2/rclc/blob/humble/rclc_examples/src/example_parameter_server.c
  // Request size limited to one parameter on Set, Get, Get types and Describe services.
  // List parameter request has no prefixes enabled nor depth.
  // Parameter description strings not allowed, rclc_add_parameter_description is disabled.
  // RCLC_PARAMETER_MAX_STRING_LENGTH = 50
  const rclc_parameter_options_t rclc_param_options = {
      .notify_changed_over_dds = false,
      .max_params = 3,
      .allow_undeclared_parameters = false,
      .low_mem_mode = true };
  
  //RCCHECK(rclc_parameter_server_init_default(&param_server, &node), cfg.ERR_UROS_PARAM);
  temp_rc = rclc_parameter_server_init_with_option(&param_server, &node, &rclc_param_options);
  if (temp_rc != RCL_RET_OK) {
    Serial.println("Micro-ROS parameter server init failed.");
    Serial.println("Make sure micro_ros_kaia library version is latest.");
    error_loop(cfg.ERR_UROS_PARAM);
  }

  RCCHECK(rclc_executor_init(&executor, &support.context,
    RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1, &allocator), cfg.ERR_UROS_EXEC);

  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub,
    &twist_msg, &twist_sub_callback, ON_NEW_DATA), cfg.ERR_UROS_EXEC);

  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server,
    on_param_changed), cfg.ERR_UROS_EXEC);;


  
  
  // TODO change to LDS scan frequency
  //RCCHECK(rclc_add_parameter(&param_server, "param_bool", RCLC_PARAMETER_BOOL), cfg.ERR_UROS_PARAM);
  //RCCHECK(rclc_add_parameter(&param_server, "param_int", RCLC_PARAMETER_INT), cfg.ERR_UROS_PARAM);
  RCCHECK(rclc_add_parameter(&param_server, cfg.UROS_PARAM_LDS_MOTOR_SPEED,
    RCLC_PARAMETER_DOUBLE), cfg.ERR_UROS_PARAM);


  
  
  // TODO change to LDS scan frequency
  //RCCHECK(rclc_parameter_set_bool(&param_server, "param_bool", false), cfg.ERR_UROS_PARAM);
  //RCCHECK(rclc_parameter_set_int(&param_server, "param_int", 10), cfg.ERR_UROS_PARAM);
  RCCHECK(rclc_parameter_set_double(&param_server, cfg.UROS_PARAM_LDS_MOTOR_SPEED,
    cfg.LDS_MOTOR_SPEED_DEFAULT), cfg.ERR_UROS_PARAM);

  //rclc_add_parameter_description(&param_server, "param_int", "Second parameter", "Only even numbers");
  //RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "param_int", -10, 120, 2), cfg.ERR_UROS_PARAM);

  //rclc_add_parameter_description(&param_server, "param_double", "Third parameter", "");
  //RCCHECK(rclc_set_parameter_read_only(&param_server, "param_double", true), cfg.ERR_UROS_PARAM);



  
  // TODO change to LDS scan frequency
  RCCHECK(rclc_add_parameter_constraint_double(&param_server, cfg.UROS_PARAM_LDS_MOTOR_SPEED,
    -1.0, 1.0, 0), cfg.ERR_UROS_PARAM);

  //bool param_bool;
  //int64_t param_int;
  //double param_double;

  //RCCHECK(rclc_parameter_get_bool(&param_server, "param_bool", &param_bool), cfg.ERR_UROS_PARAM);
  //RCCHECK(rclc_parameter_get_int(&param_server, "param_int", &param_int), cfg.ERR_UROS_PARAM);
  //RCCHECK(rclc_parameter_get_double(&param_server, "param_double", &param_double), cfg.ERR_UROS_PARAM);

  resetTelemMsg();
}

bool on_param_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
  (void) context;

  if (old_param == NULL || new_param == NULL) {
    Serial.println("old_param == NULL || new_param == NULL");
    return false;
  }

  Serial.print("Parameter ");
  Serial.print(old_param->name.data);
  Serial.print(" modified ");
  switch (old_param->value.type) {
    case RCLC_PARAMETER_BOOL:
      Serial.print(old_param->value.bool_value);
      Serial.print(" to ");
      Serial.println(new_param->value.bool_value);
      break;
    case RCLC_PARAMETER_INT:
      Serial.print(old_param->value.integer_value);
      Serial.print(" to ");
      Serial.println(new_param->value.integer_value);
      break;
    case RCLC_PARAMETER_DOUBLE:
      Serial.print(old_param->value.double_value);
      Serial.print(" to ");
      Serial.println(new_param->value.double_value);

      if (strcmp(old_param->name.data, cfg.UROS_PARAM_LDS_MOTOR_SPEED) == 0) {
        //int16_t speed_int = round((float)(new_param->value.double_value) * 255);
//        lds->setScanTargetFreqHz(new_param->value.double_value);
      }
      break;
    default:
      break;
  }

  return true;
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

  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  Serial.print(" ");

  unsigned long startMillis = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startMillis >= cfg.WIFI_CONN_TIMEOUT_MS) {
      Serial.println(" timed out");
      return false;
    }

    digitalWrite(cfg.LED_PIN, HIGH);
    delay(250);
    digitalWrite(cfg.LED_PIN, LOW);
    Serial.print('.'); // Don't use F('.'), it crashes code!!
    delay(250);
  }

  digitalWrite(cfg.LED_PIN, LOW);
  Serial.println(" connected");
  Serial.print("IP ");
  Serial.println(WiFi.localIP());
  return true;
}

void serialPrintLnNonBlocking(const String s) {
  uint16_t tx_room = (uint16_t) Serial.availableForWrite();
  if (tx_room >= s.length())
    Serial.println(s);
}

void spinTelem(bool force_pub) {
  static int telem_pub_count = 0;
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - telem_prev_pub_time_us;

  if (!force_pub && (step_time_us < telem_pub_period_us))
    return;

  publishTelem(step_time_us);
  telem_prev_pub_time_us = time_now_us;

  digitalWrite(cfg.LED_PIN, !digitalRead(cfg.LED_PIN));
  //if (++telem_pub_count % 5 == 0) {
    //Serial.print("RPM L ");
    //Serial.print(drive.getCurrentRPM(drive.MOTOR_LEFT));
    //Serial.print(" R ");
    //Serial.println(drive.getCurrentRPM(drive.MOTOR_RIGHT));
  //}

  stat_sum_spin_telem_period_us += step_time_us;
  stat_max_spin_telem_period_us = stat_max_spin_telem_period_us <= step_time_us ?
    step_time_us : stat_max_spin_telem_period_us;
  
  // How often telemetry gets published
  if (++telem_pub_count % cfg.SPIN_TELEM_STATS == 0) {
    String s = "spinTelem() period avg ";
    s = s + String(stat_sum_spin_telem_period_us / (1000*cfg.SPIN_TELEM_STATS));
    s = s + " max ";
    s = s + String(stat_max_spin_telem_period_us / 1000);
    s = s + "ms";

    float rpm = lds->getCurrentScanFreqHz();
    if (rpm >= 0) {
      s = s + ", LDS RPM ";
      s = s + String(rpm);
    }

    s = s + ", battery " + String(telem_msg.battery_mv*0.001f) + "V";
    s = s + ", RSSI " + String(telem_msg.wifi_rssi_dbm) + "dBm";
    serialPrintLnNonBlocking(s);

    stat_sum_spin_telem_period_us = 0;
    stat_max_spin_telem_period_us = 0;
  }
}

void publishTelem(unsigned long step_time_us) {
  struct timespec tv = {0, 0};
  clock_gettime(CLOCK_REALTIME, &tv);
  telem_msg.stamp.sec = tv.tv_sec;
  telem_msg.stamp.nanosec = tv.tv_nsec;

  float joint_pos_delta[drive.MOTOR_COUNT];
  float step_time = 1e-6 * (float)step_time_us;

  long rssi_dbm = WiFi.RSSI();
  rssi_dbm = rssi_dbm > 127 ? 127 : rssi_dbm;
  rssi_dbm = rssi_dbm < -128 ? -128 : rssi_dbm;
  telem_msg.wifi_rssi_dbm = (int8_t) rssi_dbm;

  unsigned int voltage_mv = analogReadMilliVolts(cfg.BAT_ADC_PIN);
  voltage_mv *= cfg.BAT_ADC_MULTIPLIER;
  if (voltage_mv < cfg.BAT_PRESENT_MV_MIN)
    voltage_mv = 0;

  telem_msg.battery_mv = (uint16_t) voltage_mv;

  //Serial.print(rssi_dbm);
  //Serial.print("dbm, ");
  //Serial.print(voltage_mv);
  //Serial.println("mV");

  for (unsigned char i = 0; i < drive.MOTOR_COUNT; i++) {
    joint[i].pos = drive.getShaftAngle(i);
    joint_pos_delta[i] = joint[i].pos - joint_prev_pos[i];
    joint[i].vel = joint_pos_delta[i] / step_time;    
    joint_prev_pos[i] = joint[i].pos;
  }

  calcOdometry(step_time_us, joint_pos_delta[drive.MOTOR_LEFT],
    joint_pos_delta[drive.MOTOR_RIGHT]);

  RCSOFTCHECK(rcl_publish(&telem_pub, &telem_msg, NULL));
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
  float d_yaw = asin((distance_left - distance_right)*cfg.wheel_base_recip);

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

void lds_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  return;

  static int i = 0;

  if ((i++ % 20 == 0) || scan_completed) {
    //Serial.print(i);
    //Serial.print('\t');
    Serial.print(angle_deg);
    Serial.print('\t');
    Serial.print(distance_mm);
    if (scan_completed) {
      Serial.print('\t');
      Serial.println(millis());
    } else
      Serial.println();
  }
}

void lds_packet_callback(uint8_t * packet, uint16_t packet_length, bool scan_completed) {
  bool packet_sent = false;
  while (packet_length-- > 0) {
    if (telem_msg.lds.size >= telem_msg.lds.capacity) {
      spinTelem(true);
      packet_sent = true;
    }
    telem_msg.lds.data[telem_msg.lds.size++] = *packet;
    packet++;
  }

  if (scan_completed && !packet_sent && (telem_msg.lds.size > 0))
    spinTelem(true); // Opional, reduce lag a little
}

void lds_motor_pin_callback(float value, LDS::lds_pin_t lds_pin) {
  /*
  Serial.print("LDS pin ");
  Serial.print(lds->pinIDToString(lds_pin));
  Serial.print(" set ");
  if (lds_pin > 0)
    Serial.print(value); // PWM value
  else
    Serial.print(lds->pinStateToString((LDS::lds_pin_state_t)value));
  Serial.print(", RPM ");
  Serial.println(lds->getCurrentScanFreqHz());
  */
  
  int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ?
    cfg.LDS_MOTOR_EN_PIN : cfg.LDS_MOTOR_PWM_PIN;

  if (value <= LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      pinMode(pin, OUTPUT);
      ledcSetup(cfg.LDS_MOTOR_PWM_CHANNEL, cfg.LDS_MOTOR_PWM_FREQ, cfg.LDS_MOTOR_PWM_BITS);
      ledcAttachPin(pin, cfg.LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<cfg.LDS_MOTOR_PWM_BITS)-1)*value;
    ledcWrite(cfg.LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

void spinPing() {
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - ping_prev_pub_time_us;
  
  if (step_time_us >= ping_pub_period_us) {
    // timeout_ms, attempts
    rmw_ret_t rc = rmw_uros_ping_agent(1, 1);
    ping_prev_pub_time_us = time_now_us;
    Serial.println(rc == RCL_RET_OK ? "Ping OK" : "Ping error");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    lds->stop();
    drive.setRPM(drive.MOTOR_RIGHT, 0);
    drive.setRPM(drive.MOTOR_LEFT, 0);
    return;
  }

  lds->loop();
  
  // Process micro-ROS callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)), cfg.ERR_UROS_SPIN);

  spinTelem(false);
  spinPing();
  updateSpeedRamp(); // update ramp less frequently?
  drive.update();
}

void resetSettings() {
  Serial.println("** Factory reset **");
  params.purge();
  digitalWrite(cfg.LED_PIN, HIGH);
  Serial.flush();
  delay(5000);

  ESP.restart();
}

bool isBootButtonPressed(uint32_t msec) {
  if (!digitalRead(0))
    Serial.println("BOOT button pressed. Keep pressing for factory reset.");
  else
    return false;

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
  telem_msg.joint.capacity = drive.MOTOR_COUNT;
  telem_msg.joint.size = drive.MOTOR_COUNT;

  telem_msg.lds.data = lds_buf;
  telem_msg.lds.capacity = cfg.LDS_BUF_LEN;
  telem_msg.lds.size = 0;

  for (int i = 0; i < drive.MOTOR_COUNT; i++) {
    joint[i].pos = 0;
    joint[i].vel = 0;
    joint_prev_pos[i] = 0;
  }

  telem_msg.battery_mv = 0;
  telem_msg.wifi_rssi_dbm = 0;
}

void syncRosTime() {
  const int timeout_ms = cfg.UROS_TIME_SYNC_TIMEOUT_MS;

  Serial.print("Syncing time ... ");
  RCCHECK(rmw_uros_sync_session(timeout_ms), cfg.ERR_UROS_TIME_SYNC);
  // https://micro.ros.org/docs/api/rmw/
  int64_t time_ms = rmw_uros_epoch_millis();
  
  if (time_ms > 0) {
    time_t time_seconds = time_ms*0.001;
    time_t time_micro_seconds = (time_ms - time_seconds*1000)*1000; 
    
    // https://gist.github.com/igrr/d7db8a78170bf6981f2e606b42c4361c
    setenv("TZ", "GMT0", 1);
    tzset();
    
    // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-time.c
    timeval epoch = {time_seconds, time_micro_seconds};
    if (settimeofday((const timeval*)&epoch, NULL) != 0)
      Serial.println("settimeofday() error");
    else
      Serial.println("OK");
  } else {
    Serial.print("rmw_uros_sync_session() failed, error code: ");
    Serial.println((int) time_ms);
  }
}

//static inline void logMsgInfo(char* msg) {
//  logMsg(msg, rcl_interfaces__msg__Log__INFO);
//}

void logMsg(char* msg, uint8_t severity_level) {
  if (WiFi.status() != WL_CONNECTED) {
    rcl_interfaces__msg__Log msgLog;
    // https://docs.ros2.org/foxy/api/rcl_interfaces/msg/Log.html
    // builtin_interfaces__msg__Time stamp;
    struct timespec tv = {0, 0};
    clock_gettime(CLOCK_REALTIME, &tv);
    msgLog.stamp.sec = tv.tv_sec;
    msgLog.stamp.nanosec = tv.tv_nsec;
    
    msgLog.level = severity_level;
    msgLog.name.data = (char *)params.get(cfg.PARAM_ROBOT_MODEL_NAME);
    msgLog.name.size = strlen(msgLog.name.data);
    msgLog.msg.data = msg;
    msgLog.msg.size = strlen(msgLog.msg.data);
    //char fileName[] = __FILE__;
    msgLog.file.data = (char*)""; // Source code file name
    msgLog.file.size = strlen(msgLog.file.data);
    msgLog.function.data = (char*)""; // Source code function name
    msgLog.function.size = strlen(msgLog.function.data);
    msgLog.line = 0; // Source code line number
    RCSOFTCHECK(rcl_publish(&log_pub, &msgLog, NULL));
  }
  
  String s = "UNDEFINED";
  switch(severity_level) {
    case rcl_interfaces__msg__Log__INFO:
      s = "INFO";
      break;
    case rcl_interfaces__msg__Log__FATAL:
      s = "FATAL";
      break;
    case rcl_interfaces__msg__Log__DEBUG:
      s = "DEBUG";
      break;
    case rcl_interfaces__msg__Log__ERROR:
      s = "ERROR";
      break;
    case rcl_interfaces__msg__Log__WARN:
      s = "WARN";
      break;
  }
  Serial.print("LOG_");
  Serial.print(s);
  Serial.print(": ");
  Serial.println(msg);
}

void lds_info_callback(LDS::info_t code, String info) {
  Serial.print("LDS info ");
  Serial.print(lds->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lds_error_callback(LDS::result_t code, String aux_info) {
  if (code != LDS::ERROR_NOT_READY) {
    String s = "LDS ";
    s = s + String(lds->resultCodeToString(code));

    if (aux_info.length() > 0) {
      s = s + ": ";
      s = s + String(aux_info);
    }
    serialPrintLnNonBlocking(s);
  }
}

void setupLDS() {
  const char * model = params.get(cfg.PARAM_LDS_MODEL);
  Serial.print("LDS model ");
  Serial.print(model);

  if (strcmp(model, "NEATO XV11") == 0) {
    lds = new LDS_NEATO_XV11();
  } else {
    if (strcmp(model, "SLAMTEC RPLIDAR A1") == 0) {
      lds = new LDS_RPLIDAR_A1();
    } else {
      if (strcmp(model, "LDS02RR") == 0) {
        lds = new LDS_LDS02RR();
      } else {
        if (strcmp(model, "YDLIDAR X2/X2L") == 0) {
          lds = new LDS_YDLIDAR_X2_X2L();
        } else {
          if (strcmp(model, "YDLIDAR X3") == 0) {
            lds = new LDS_YDLIDAR_X3();
          } else {
            if (strcmp(model, "YDLIDAR X3 PRO") == 0) {
              lds = new LDS_YDLIDAR_X3_PRO();
            } else {
              if (strcmp(model, "3IROBOTIX DELTA 2G") == 0) {
                lds = new LDS_DELTA_2G();
              } else {
                if (strcmp(model, "3IROBOTIX DELTA 2A 115200") == 0) {
                  lds = new LDS_DELTA_2A_115200();
                } else {
                  if (strcmp(model, "LDLIDAR DELTA 2A") == 0) {
                    lds = new LDS_DELTA_2A_230400();
                  } else {
                    if (strcmp(model, "LDLIDAR DELTA 2B") == 0) {
                      lds = new LDS_DELTA_2B();
                    } else {
                      if (strcmp(model, "LDLIDAR LD14P") == 0) {
                        lds = new LDS_LDLIDAR_LD14P();
                      } else {
                        if (strcmp(model, "YDLIDAR X4") != 0)
                          Serial.print(" not recognized, defaulting to YDLIDAR X4");
                        lds = new LDS_YDLIDAR_X4();
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  Serial.println();
    
  lds->setScanPointCallback(lds_scan_point_callback);
  lds->setPacketCallback(lds_packet_callback);
  lds->setSerialWriteCallback(lds_serial_write_callback);
  lds->setSerialReadCallback(lds_serial_read_callback);
  lds->setMotorPinCallback(lds_motor_pin_callback);
  lds->setInfoCallback(lds_info_callback);
  lds->setErrorCallback(lds_error_callback);

  Serial.print("LDS RX buffer size "); // default 128 hw + 256 sw
  Serial.flush();
  Serial.print(LdSerial.setRxBufferSize(cfg.LDS_SERIAL_RX_BUF_LEN)); // before .begin()
  uint32_t baud_rate = lds->getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);

  LdSerial.begin(baud_rate);
  lds->init();
  //while (LdSerial.read() >= 0);  

  lds->stop();
}

LDS::result_t startLDS() {  
  LDS::result_t result = lds->start();
  Serial.print("startLDS() result: ");
  Serial.println(lds->resultCodeToString(result));

  if (result < 0)
    Serial.println("Is the LiDAR/LDS connected to ESP32 and powerd up?");

  return result;
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
  lds->stop();

  char buffer[40];
  sprintf(buffer, "Error code %d", n_blinks);  
  logMsg(buffer, rcl_interfaces__msg__Log__FATAL);

  blink_error_code(n_blinks);

  Serial.println("Rebooting...");
  Serial.flush();

  ESP.restart();
}
