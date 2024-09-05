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

#include <micro_ros_kaia.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
//#include <rmw_microros/discovery.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry2.h>
#include <geometry_msgs/msg/twist.h>
//#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
#include "robot_config.h"
#include "lds_all_models.h"
#include "motors.h"
#include "param_file.h"
#include "esp_mac.h"

extern MotorController motorLeft, motorRight;
extern CONFIG cfg;
extern LDS *lidar;
extern PARAM_FILE params;
bool ros_config_params_changed = false;
bool suppress_param_log_print = false;

rcl_publisher_t telem_pub;
rcl_publisher_t log_pub;
//rcl_publisher_t diag_pub;
rcl_subscription_t twist_sub;
kaiaai_msgs__msg__KaiaaiTelemetry2 telem_msg;
geometry_msgs__msg__Twist twist_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_parameter_server_t param_server;

#define RCL_RET(fn) { rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK)return temp_rc;}
#define RCL_ERR(fn,E) { rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK)return E;}
#define RCL_PAR(fn) { rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK)return CONFIG::ERR_UROS_PARAM;}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK)Serial.println("RCSOFTCHECK failed");}

bool on_ros_param_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
  (void) context;

  if (old_param == NULL || new_param == NULL) {
    Serial.println("old_param == NULL || new_param == NULL");
    return false;
  }

  switch (old_param->value.type) {
    case RCLC_PARAMETER_BOOL:
      if (old_param->value.bool_value == new_param->value.bool_value)
        break;
      if (!suppress_param_log_print) {
        Serial.print("Parameter ");
        Serial.print(old_param->name.data);
        Serial.print(" modified ");
        Serial.print(old_param->value.bool_value);
        Serial.print(" to ");
        Serial.println(new_param->value.bool_value);
      }
      if (strcmp(old_param->name.data, cfg.UROS_PARAM_MOTOR_PID_ON_ERROR) == 0) {
        motorLeft.setPIDOnError(new_param->value.bool_value);
        motorRight.setPIDOnError(new_param->value.bool_value);
        ros_config_params_changed = true;
      }
      break;
    case RCLC_PARAMETER_INT:
      if (old_param->value.integer_value == new_param->value.integer_value)
        break;
      if (!suppress_param_log_print) {
        Serial.print("Parameter ");
        Serial.print(old_param->name.data);
        Serial.print(" modified ");
        Serial.print(old_param->value.integer_value);
        Serial.print(" to ");
        Serial.println(new_param->value.integer_value);
      }
      break;
    case RCLC_PARAMETER_DOUBLE:
      if (old_param->value.double_value == new_param->value.double_value)
        break;
      if (!suppress_param_log_print) {
        Serial.print("Parameter ");
        Serial.print(old_param->name.data);
        Serial.print(" modified ");
        Serial.print(old_param->value.double_value);
        Serial.print(" to ");
        Serial.println(new_param->value.double_value, 10);
      }
      if (strcmp(old_param->name.data, cfg.UROS_PARAM_LIDAR_SCAN_FREQ_TARGET) == 0) {
        lidar->setScanTargetFreqHz(float(new_param->value.double_value));
        ros_config_params_changed = true;
      } else if (strcmp(old_param->name.data, cfg.UROS_PARAM_MOTOR_ENCODER_PPR) == 0) {
        // TODO copy to config
        motorLeft.setEncoderPPR(float(new_param->value.double_value));
        motorRight.setEncoderPPR(float(new_param->value.double_value));
        ros_config_params_changed = true;
      } else if (strcmp(old_param->name.data, cfg.UROS_PARAM_MAX_WHEEL_ACCEL) == 0) {
        // TODO copy to config
        cfg.setMaxWheelAccel(float(new_param->value.double_value));
        ros_config_params_changed = true;
      } else if (strcmp(old_param->name.data, cfg.UROS_PARAM_MOTOR_PID_KP) == 0) {
        // TODO copy to config
        motorLeft.setPIDKp(float(new_param->value.double_value));
        motorRight.setPIDKp(float(new_param->value.double_value));
        ros_config_params_changed = true;
      } else if (strcmp(old_param->name.data, cfg.UROS_PARAM_MOTOR_PID_KI) == 0) {
        // TODO copy to config
        motorLeft.setPIDKi(float(new_param->value.double_value));
        motorRight.setPIDKi(float(new_param->value.double_value));
        ros_config_params_changed = true;
      } else if (strcmp(old_param->name.data, cfg.UROS_PARAM_MOTOR_PID_KD) == 0) {
        // TODO copy to config
        motorLeft.setPIDKd(float(new_param->value.double_value));
        motorRight.setPIDKd(float(new_param->value.double_value));
        ros_config_params_changed = true;
      } else if (strcmp(old_param->name.data, cfg.UROS_PARAM_MOTOR_PID_PERIOD) == 0) {
        motorLeft.setPIDPeriod(float(new_param->value.double_value));
        motorRight.setPIDPeriod(float(new_param->value.double_value));
        ros_config_params_changed = true;
      }
      break;
    default:
      break;
  }

  return true;
}

CONFIG::error_blink_count syncRosTime() {
  const int timeout_ms = cfg.UROS_TIME_SYNC_TIMEOUT_MS;

  Serial.print("Syncing time ... ");
  RCL_ERR(rmw_uros_sync_session(timeout_ms), CONFIG::ERR_UROS_TIME_SYNC);
  // https://micro.ros.org/docs/api/rmw/
  int64_t time_ms = rmw_uros_epoch_millis();
  
  if (time_ms > 0) {
    time_t time_seconds = time_ms*0.001;
    suseconds_t time_micro_seconds = (time_ms - time_seconds*1000)*1000; 
    
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

  return CONFIG::ERR_NONE;
}

CONFIG::error_blink_count setupMicroROS(rclc_subscription_callback_t twist_sub_callback) {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCL_ERR(rcl_init_options_init(&init_options, allocator), CONFIG::ERR_UROS_INIT);

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

  uint8_t mac[6];
  //esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    Serial.print("Error reading efuse MAC");
  }

  uint32_t client_key = mac[1]<<(3*8) | mac[2]<<(2*8) | mac[3]<<(1*8) | mac[4];
  client_key = client_key<<(8-2) | mac[5]>>2;  // TODO multiple bots
  RCL_ERR(rmw_uros_options_set_client_key(client_key, rmw_options), CONFIG::ERR_UROS_INIT);

  rcl_ret_t temp_rc;

  while(true) {
    digitalWrite(cfg.LED_PIN, !digitalRead(cfg.LED_PIN));
    Serial.print(F("Connecting to Micro-ROS agent "));
    Serial.print(params.get(cfg.PARAM_DEST_IP));
    Serial.print(" ... ");
  
    //rclc_support_init(&support, 0, NULL, &allocator);
    temp_rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (temp_rc != RCL_RET_OK) {
      //Serial.println("failed or timed out");
      Serial.println();
      //return CONFIG::ERR_UROS_AGENT_CONN;
      continue;
    }
    break;
  }
  Serial.println("success");

  syncRosTime();
  printCurrentTime();

  // https://micro.ros.org/docs/tutorials/programming_rcl_rclc/node/
  RCL_ERR(rclc_node_init_default(&node, CONFIG::UROS_NODE_NAME, "", &support), CONFIG::ERR_UROS_NODE);

  Serial.print("micro-ROS client key 0x");
  Serial.print(client_key, HEX);
  Serial.print("; ROS2 node /");
  Serial.println(cfg.UROS_NODE_NAME);

  RCL_ERR(rclc_subscription_init_default(&twist_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), cfg.UROS_CMD_VEL_TOPIC_NAME),
    CONFIG::ERR_UROS_PUBSUB);

  RCL_ERR(rclc_publisher_init_best_effort(&telem_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(kaiaai_msgs, msg, KaiaaiTelemetry2), cfg.UROS_TELEM_TOPIC_NAME),
    CONFIG::ERR_UROS_PUBSUB);

  RCL_ERR(rclc_publisher_init_default(&log_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), cfg.UROS_LOG_TOPIC_NAME),
    CONFIG::ERR_UROS_PUBSUB);
/*
  RCL_ERR(rclc_publisher_init_default(&diag_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray), cfg.UROS_DIAG_TOPIC_NAME),
    CONFIG::ERR_UROS_PUBSUB);
*/
  // https://github.com/ros2/rclc/blob/humble/rclc_examples/src/example_parameter_server.c
  // Request size limited to one parameter on Set, Get, Get types and Describe services.
  // List parameter request has no prefixes enabled nor depth.
  // Parameter description strings not allowed, rclc_add_parameter_description is disabled.
  // RCLC_PARAMETER_MAX_STRING_LENGTH = 50
  const rclc_parameter_options_t rclc_param_options = {
      .notify_changed_over_dds = false,
      .max_params = cfg.UROS_PARAM_COUNT,
      .allow_undeclared_parameters = false,
      .low_mem_mode = true };
  
  //RCL_ERR(rclc_parameter_server_init_default(&param_server, &node), CONFIG::ERR_UROS_PARAM);
  temp_rc = rclc_parameter_server_init_with_option(&param_server, &node, &rclc_param_options);
  if (temp_rc != RCL_RET_OK) {
    Serial.println("Micro-ROS parameter server init failed.");
    Serial.println("Make sure micro_ros_kaia library version is latest.");
    return CONFIG::ERR_UROS_PARAM;
  }

  RCL_ERR(rclc_executor_init(&executor, &support.context,
    RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1, &allocator), CONFIG::ERR_UROS_EXEC);

  RCL_ERR(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg,
    twist_sub_callback, ON_NEW_DATA), CONFIG::ERR_UROS_EXEC);

  RCL_ERR(rclc_executor_add_parameter_server(&executor, &param_server,
    on_ros_param_changed), CONFIG::ERR_UROS_EXEC);

  return CONFIG::ERR_NONE;
}

CONFIG::error_blink_count addROSParams() {
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_LIDAR_SCAN_FREQ_TARGET, RCLC_PARAMETER_DOUBLE));
  //RCL_PAR(rclc_add_parameter_constraint_double(&param_server, cfg.UROS_PARAM_LIDAR_SCAN_FREQ_TARGET,
  // -1.0, 1.0, 0));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_PID_KP, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_PID_KI, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_PID_KD, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_PID_ON_ERROR, RCLC_PARAMETER_BOOL));
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_PID_PERIOD, RCLC_PARAMETER_DOUBLE));

  // Read-only
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_LIDAR_SCAN_FREQ_NOW, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_LIDAR_SCAN_FREQ_NOW, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_ENCODER_NOW, RCLC_PARAMETER_INT));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_ENCODER_NOW, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_ENCODER_NOW, RCLC_PARAMETER_INT));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_ENCODER_NOW, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_ENCODER_PPR, RCLC_PARAMETER_DOUBLE));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_ENCODER_TPR, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_ENCODER_TPR, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_RPM_MAX_DERATED, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_RPM_MAX_DERATED, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_RPM_NOW, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_RPM_NOW, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_RPM_NOW, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_RPM_NOW, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_RPM_TARGET, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_RPM_TARGET, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_RPM_TARGET, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_RPM_TARGET, true));

  // TODO positive value
  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MAX_WHEEL_ACCEL, RCLC_PARAMETER_DOUBLE));
//  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MAX_WHEEL_ACCEL, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_BASE_DIA, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_BASE_DIA, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_BASE_WHEEL_TRACK, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_BASE_WHEEL_TRACK, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_BASE_WHEEL_DIA, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_BASE_WHEEL_DIA, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_PWM_NOW, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_LEFT_PWM_NOW, true));

  RCL_PAR(rclc_add_parameter(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_PWM_NOW, RCLC_PARAMETER_DOUBLE));
  RCL_PAR(rclc_set_parameter_read_only(&param_server, cfg.UROS_PARAM_MOTOR_RIGHT_PWM_NOW, true));

  //rclc_add_parameter_description(&param_server, "param_int", "Second parameter", "Only even numbers");
  //RCL_PAR(rclc_add_parameter_constraint_integer(&param_server, "param_int", -10, 120, 2));
  //rclc_add_parameter_description(&param_server, "param_double", "Third parameter", "");
  
  return CONFIG::ERR_NONE;
}

rcl_ret_t update_double(const char * param_name, double value) {
  double current_value;
  rcl_ret_t rc;

  rc = rclc_parameter_get_double(&param_server, param_name, &current_value);
  if (rc != RCL_RET_OK)
    return rc;

  if (current_value == value)
    return RCL_RET_OK;

  return rclc_parameter_set_double(&param_server, param_name, value);
}

rcl_ret_t update_int(const char * param_name, int64_t value) {
  int64_t current_value;
  rcl_ret_t rc;

  rc = rclc_parameter_get_int(&param_server, param_name, &current_value);
  if (rc != RCL_RET_OK)
    return rc;

  if (current_value == value)
    return RCL_RET_OK;

  return rclc_parameter_set_int(&param_server, param_name, value);
}

rcl_ret_t update_bool(const char * param_name, bool value) {
  bool current_value;
  rcl_ret_t rc;

  rc = rclc_parameter_get_bool(&param_server, param_name, &current_value);
  if (rc != RCL_RET_OK)
    return rc;

  if (current_value == value)
    return RCL_RET_OK;

  return rclc_parameter_set_bool(&param_server, param_name, value);
}

CONFIG::error_blink_count updateROSRealTimeParams() {
  // Frequently changed
  suppress_param_log_print = true;
  RCL_PAR(update_double(cfg.UROS_PARAM_LIDAR_SCAN_FREQ_NOW, lidar->getCurrentScanFreqHz()));

  RCL_PAR(update_int(cfg.UROS_PARAM_MOTOR_LEFT_ENCODER_NOW, motorLeft.getEncoderValue()));
  RCL_PAR(update_int(cfg.UROS_PARAM_MOTOR_RIGHT_ENCODER_NOW, motorRight.getEncoderValue()));

  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_LEFT_RPM_NOW, motorLeft.getCurrentRPM()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_RIGHT_RPM_NOW, motorRight.getCurrentRPM()));

  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_LEFT_RPM_TARGET, motorLeft.getTargetRPM()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_RIGHT_RPM_TARGET, motorRight.getTargetRPM()));

  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_LEFT_PWM_NOW, motorLeft.getCurrentPWM()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_RIGHT_PWM_NOW, motorRight.getCurrentPWM()));

  //Serial.print("L ");
  //Serial.print(motorLeft.getEncoderValue());
  //Serial.print("\tR ");
  //Serial.println(motorRight.getEncoderValue());

  suppress_param_log_print = false;
  return CONFIG::ERR_NONE;
}

CONFIG::error_blink_count updateROSConfigParams() {
  suppress_param_log_print = true;

  RCL_PAR(update_double(cfg.UROS_PARAM_LIDAR_SCAN_FREQ_TARGET, lidar->getTargetScanFreqHz()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_ENCODER_PPR, motorLeft.getEncoderPPR()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_ENCODER_TPR, motorLeft.getEncoderTPR()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_RPM_MAX_DERATED, motorLeft.getMaxRPM()));

  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_PID_KP, motorLeft.getPIDKp()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_PID_KI, motorLeft.getPIDKi()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_PID_KD, motorLeft.getPIDKd()));
  RCL_PAR(update_bool(cfg.UROS_PARAM_MOTOR_PID_ON_ERROR, motorLeft.getPIDOnError()));
  RCL_PAR(update_double(cfg.UROS_PARAM_MOTOR_PID_PERIOD, motorLeft.getPIDPeriod()));

  RCL_PAR(update_double(cfg.UROS_PARAM_MAX_WHEEL_ACCEL, cfg.base_wheel_accel_max));
  RCL_PAR(update_double(cfg.UROS_PARAM_BASE_DIA, params.getAsFloat(cfg.PARAM_BASE_DIA)));
  RCL_PAR(update_double(cfg.UROS_PARAM_BASE_WHEEL_TRACK, params.getAsFloat(cfg.PARAM_BASE_WHEEL_TRACK)));
  RCL_PAR(update_double(cfg.UROS_PARAM_BASE_WHEEL_DIA, params.getAsFloat(cfg.PARAM_BASE_WHEEL_DIA)));

  suppress_param_log_print = false;
  return CONFIG::ERR_NONE;
}

//void logMsgInfo(char* msg) {
//  logMsg(msg, rcl_interfaces__msg__Log__INFO);
//}

void logMsg(char* msg, uint8_t severity_level) {
  if (WiFi.status() == WL_CONNECTED) {
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

/*
void pubDiagnostics() {
  // github.com/ros2/common_interfaces/tree/rolling/diagnostic_msgs/
  const int STATUS_COUNT = 1;
  const int KEY_VALUE_COUNT = 1;
  diagnostic_msgs__msg__DiagnosticArray msgDiagArray;
  diagnostic_msgs__msg__DiagnosticStatus msgDiagStatus[STATUS_COUNT];
  diagnostic_msgs__msg__KeyValue msgKeyValue[KEY_VALUE_COUNT];

  struct timespec tv = {0, 0};
  clock_gettime(CLOCK_REALTIME, &tv);
  msgDiagArray.header.stamp.sec = tv.tv_sec;
  msgDiagArray.header.stamp.nanosec = tv.tv_nsec;

  msgDiagArray.status.data = msgDiagStatus;
  msgDiagArray.status.capacity = STATUS_COUNT;
  msgDiagArray.status.size = STATUS_COUNT;

  // OK, WARN, ERROR, STALE
  msgDiagStatus[0].level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  msgDiagStatus[0].name.data = (char*)"LIDAR";
  msgDiagStatus[0].name.size = strlen(msgDiagStatus[0].name.data);
  //msgDiagStatus[0].message
  //msgDiagStatus[0].hardware_id

  msgDiagStatus[0].values.data = msgKeyValue;
  msgDiagStatus[0].values.capacity = KEY_VALUE_COUNT;
  msgDiagStatus[0].values.size = KEY_VALUE_COUNT;

  msgKeyValue[0].key.data = (char*)"MODEL";
  msgKeyValue[0].key.size = strlen(msgKeyValue[0].key.data);

  msgKeyValue[0].value.data = (char *) params.get(cfg.PARAM_LIDAR_MODEL);
  msgKeyValue[0].value.size = strlen(msgKeyValue[0].value.data);

  RCSOFTCHECK(rcl_publish(&diag_pub, &msgDiagArray, NULL));
//  rcl_ret_t ret = rcl_publish(&diag_pub, &msgDiagArray, NULL);
//  return ret == RCL_RET_OK;
}
*/
