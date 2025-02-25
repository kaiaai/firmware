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

#include "lds_all_models.h"
#include "robot_config.h"
#include "util.h"
#include <micro_ros_kaia.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry2.h>
#include <HardwareSerial.h>

extern CONFIG cfg;
extern kaiaai_msgs__msg__KaiaaiTelemetry2 telem_msg;
LDS *lidar;
HardwareSerial LdSerial(2);

void spinTelem(bool);

class LDS_NONE : public LDS {
  public:
    virtual void init() override {}
    virtual result_t start() override { return ERROR_NOT_IMPLEMENTED; }
    virtual result_t stop() override { return ERROR_NOT_IMPLEMENTED; }
    virtual void loop() override {}
    virtual uint32_t getSerialBaudRate() { return 0; }
    virtual float getCurrentScanFreqHz() override { return -1.0f; }
    virtual float getTargetScanFreqHz() override { return -1.0f; }
    virtual int getSamplingRateHz() override { return -1.0f; }
    virtual bool isActive() override { return false; }
    virtual const char* getModelName() override { return "NONE"; }

    virtual result_t setScanTargetFreqHz(float freq) override { return ERROR_NOT_IMPLEMENTED; }
};

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LdSerial.write(buffer, length);
}

int lidar_serial_read_callback() {
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

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
/*
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
*/
/*
  if (scan_completed)
    Serial.println();
  
  Serial.print(angle_deg);
  Serial.print('\t');
  Serial.print(distance_mm);
  Serial.print('\t');
  Serial.println(quality);
*/
}

void lidar_packet_callback(uint8_t * packet, uint16_t packet_length, bool scan_completed) {
  bool packet_sent = false;
//  Serial.println('-');
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

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lds_pin) {
/*
  Serial.print("LiDAR pin ");
  Serial.print(lidar->pinIDToString(lds_pin));
  Serial.print(" set ");
  if (lds_pin > 0)
    Serial.print(value); // PWM value
  else
    Serial.print(lidar->pinStateToString((LDS::lds_pin_state_t)value));
  Serial.print(", RPM ");
  Serial.println(lidar->getCurrentScanFreqHz());
*/
  int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ?
    cfg.lidar_gpio_en : cfg.lidar_gpio_pwm;

  if (pin == 255)
    return;

  if (value <= (float)LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
      #if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(cfg.LIDAR_PWM_CHANNEL, cfg.LIDAR_PWM_FREQ, cfg.LIDAR_PWM_BITS);
      ledcAttachPin(pin, cfg.LIDAR_PWM_CHANNEL);
      #else      
      if (!ledcAttachChannel(pin, cfg.LIDAR_PWM_FREQ,
        cfg.LIDAR_PWM_BITS, cfg.LIDAR_PWM_CHANNEL))
        Serial.println("lidar_motor_pin_callback() ledcAttachChannel() error");
      #endif
    } else
      setPinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < (float)LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<cfg.LIDAR_PWM_BITS)-1)*value;
    #if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(cfg.LIDAR_PWM_CHANNEL, pwm_value);
    #else
    ledcWriteChannel(cfg.LIDAR_PWM_CHANNEL, pwm_value);
    #endif
  }
}

void lidar_info_callback(LDS::info_t code, const String info) {
  Serial.print("LiDAR info ");
  Serial.print(lidar->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, const String aux_info) {
//  return;

  if (code != LDS::ERROR_NOT_READY) {
    String s = "LiDAR ";
    s = s + String(lidar->resultCodeToString(code));

    if (aux_info.length() > 0) {
      s = s + ": ";
      s = s + String(aux_info);
    }
    printlnNB(s);
  }
}

void setupLIDAR() {
  if (cfg.lidar_gpio_pwm != 255) {
    #if ESP_IDF_VERSION_MAJOR >= 5
    if (!ledcAttachChannel(cfg.lidar_gpio_pwm, cfg.LIDAR_PWM_FREQ,
      cfg.LIDAR_PWM_BITS, cfg.LIDAR_PWM_CHANNEL))
      Serial.println("setupLIDAR() ledcAttachChannel() error");
    #else
    if (!ledcSetup(cfg.LIDAR_PWM_CHANNEL, cfg.LIDAR_PWM_FREQ, cfg.LIDAR_PWM_BITS))
      Serial.println("setupLIDAR() ledcSetup() error");
    #endif
  }

  Serial.print("LIDAR model ");
  Serial.print(cfg.lidar_model);

  if (cfg.lidar_model == "NEATO XV11") {
    lidar = new LDS_NEATO_XV11();
  } else {
    if (cfg.lidar_model == "SLAMTEC RPLIDAR A1") {
      lidar = new LDS_RPLIDAR_A1();
    } else {
      if (cfg.lidar_model == "XIAOMI LDS02RR") {
        lidar = new LDS_LDS02RR();
      } else {
        if (cfg.lidar_model == "YDLIDAR SCL") {
          lidar = new LDS_YDLIDAR_SCL();
        } else {
          if (cfg.lidar_model == "YDLIDAR X2" || cfg.lidar_model == "YDLIDAR X2L") {
            lidar = new LDS_YDLIDAR_X2_X2L();
          } else {
            if (cfg.lidar_model == "YDLIDAR X3") {
              lidar = new LDS_YDLIDAR_X3();
            } else {
              if (cfg.lidar_model == "YDLIDAR X3 PRO") {
                lidar = new LDS_YDLIDAR_X3_PRO();
              } else {
                if (cfg.lidar_model == "3IROBOTIX DELTA-2G") {
                  lidar = new LDS_DELTA_2G();
                } else {
                  if (cfg.lidar_model == "3IROBOTIX DELTA-2A 115200") {
                    lidar = new LDS_DELTA_2A_115200();
                  } else {
                    if (cfg.lidar_model == "3IROBOTIX DELTA-2A") {
                      lidar = new LDS_DELTA_2A_230400();
                    } else {
                      if (cfg.lidar_model == "3IROBOTIX DELTA-2B") {
                        lidar = new LDS_DELTA_2B();
                      } else {
                        if (cfg.lidar_model == "LDROBOT LD14P") {
                          lidar = new LDS_LDROBOT_LD14P();
                        } else {
                          if (cfg.lidar_model == "YDLIDAR X4 PRO") {
                            lidar = new LDS_YDLIDAR_X4_PRO();
                          } else {
                            if (cfg.lidar_model != "YDLIDAR X4")
                              Serial.print(" not recognized, defaulting to YDLIDAR X4");
                            lidar = new LDS_YDLIDAR_X4();
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
    }
  }
  Serial.println();

  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);

  Serial.print("LIDAR RX buffer size "); // default 128 hw + 256 sw
  Serial.flush();
  Serial.print(LdSerial.setRxBufferSize(cfg.LIDAR_SERIAL_RX_BUF_LEN)); // before .begin()
  uint32_t baud_rate = lidar->getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);

  LdSerial.begin(baud_rate, SERIAL_8N1, cfg.lidar_gpio_tx, cfg.lidar_gpio_rx);
  gpio_set_drive_capability((gpio_num_t) cfg.lidar_gpio_rx, GPIO_DRIVE_CAP_0);

  lidar->init();
  lidar->stop();
}

LDS::result_t startLIDAR() {  
  LDS::result_t result = lidar->start();
  Serial.print("startLIDAR() result: ");
  Serial.println(lidar->resultCodeToString(result));

  if (result < 0)
    Serial.println("Is the LiDAR connected to ESP32 and powered up?");

  lidar->setScanTargetFreqHz(cfg.lidar_scan_freq_target);
  return result;
}
