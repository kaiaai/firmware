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

#include <Arduino.h>
#include "robot_config.h"
#include <SPIFFS.h>
#include <esp_wifi.h>

extern CONFIG cfg;

bool write_file(const char* FILE_PATH, const char* text) {
  Serial.print("Writing file ");
  Serial.print(FILE_PATH);

  File file = SPIFFS.open(FILE_PATH, FILE_WRITE);
  if (!file) {
    Serial.print(" - file open failed");
    return false;
  }

  bool success = true;
  if (text != NULL) {
    success = file.print(text);
    if (!success)
      Serial.println(" - write failed");
  }

  file.close();
  return success;
}

inline void digiWrite(uint8_t gpio, uint8_t level, bool invert) {
  if (gpio == cfg.UNDEFINED_GPIO)
    return;

  if (invert)
    level = level == HIGH ? LOW : HIGH;

  digitalWrite(gpio, level);
}

inline bool digiRead(uint8_t gpio, bool invert) {
  if (gpio == cfg.UNDEFINED_GPIO)
    return invert;

  return ((bool) digitalRead(gpio)) != invert;
}

inline bool setPinDrive(uint8_t pin, gpio_drive_cap_t strength = GPIO_DRIVE_CAP_0) {
  if (pin == cfg.UNDEFINED_GPIO)
    return false;

  esp_err_t err = gpio_set_drive_capability((gpio_num_t) pin, strength);
  return err == ESP_OK;
}

inline bool setPinMode(uint8_t pin, uint8_t mode,
  gpio_drive_cap_t strength = GPIO_DRIVE_CAP_0) {

  if (pin == cfg.UNDEFINED_GPIO)
    return false;
  
  pinMode(pin, mode);
  return mode == INPUT ? true : setPinDrive(pin, strength);
}

void blink(unsigned int delay_ms, unsigned int count) {
  for (unsigned int i = 0; i < count; i++) {
    digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
    delay(delay_ms);
    digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);
    delay(delay_ms);
  }
  digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
}

float absMin(float a, float b_abs) {
  float a_abs = abs(a);
  float min_abs = min(a_abs, b_abs);

  return a >= 0 ? min_abs : -min_abs;
}

void delaySpin(unsigned long msec) {
  unsigned long time_msec = millis();
  while(millis() - time_msec < msec) {
    yield();
  }
}

void printCurrentTime() {
  // prints time correctly, even after power-up
  char strftime_buf[64];
  time_t now;

  setenv("TZ", "CEST-1CET,M3.2.0/2:00:00,M11.1.0/2:00:00", 1); // for local time
  tzset();
  time(&now);

  struct tm *tt = gmtime(&now);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", tt);
  Serial.print("UTC time ");
  Serial.println(strftime_buf);

  //localtime_r(&now, tt);
  //strftime(strftime_buf, sizeof(strftime_buf), "%c", tt);
  //Serial.print("The current local date/time according to ESP32: ");
  //Serial.println(strftime_buf);
}

void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
  float *speed_right, float *speed_left);

void delayYield(unsigned long msec) {
    unsigned long time_msec = millis();
    while(millis() - time_msec < msec)
      yield();
}

const String micro_ros_error_string(int err) {
  String errCode = String(err);
  switch (err)
  {
    case 0 : return ("RMW_RET_OK");
    case 1 : return ("RMW_RET_ERROR");
    case 2 : return ("RMW_RET_TIMEOUT");
    case 23 : return ("RMW_GID_STORAGE_SIZE");
    default : return errCode;
  }
}

void printByteAsHex(uint8_t b) {
  if (b < 16)
    Serial.print('0');
  Serial.print(b, HEX);
}

void printBytesAsHex(const uint8_t * buffer, uint16_t length) {
  if (length == 0)
    return;

  for (uint16_t i = 0; i < length; i++) {
    printByteAsHex(buffer[i]);
    Serial.print(' ');
  }
}

void printlnNB(const String & s = "") { // non-blocking
  uint16_t tx_room = (uint16_t) Serial.availableForWrite();
  if (tx_room >= s.length())
    Serial.println(s);
}

void printNB(const String & s) { // non-blocking
  uint16_t tx_room = (uint16_t) Serial.availableForWrite();
  if (tx_room >= s.length())
    Serial.print(s);
}

void idle() {
  while(true)
    delay(0);
}

void printWiFiChannel() {
  uint8_t primary_ch;
  wifi_second_chan_t secondary_ch;
  esp_err_t err = esp_wifi_get_channel(&primary_ch, &secondary_ch);
  if (err == ESP_OK) {
    Serial.print("Primary AP channel ");
    Serial.println(primary_ch);

    Serial.print("Secondary channel ");
    switch(secondary_ch) {
      case WIFI_SECOND_CHAN_NONE:
        Serial.print("NONE/HT20");
        break;
      case WIFI_SECOND_CHAN_ABOVE:
        Serial.print("ABOVE/HT40");
        break;
      case WIFI_SECOND_CHAN_BELOW:
        Serial.print("BELOW/HT40");
        break;
      default:
        Serial.print("Unknown");
        break;
    }
    Serial.print(" ");
  } else {
    Serial.print("esp_wifi_get_channel() failed ");
    switch(err) {
      case ESP_ERR_WIFI_CONN:
        Serial.println("ESP_ERR_WIFI_CONN");
        break;
      case ESP_ERR_WIFI_NOT_INIT:
        Serial.println("ESP_ERR_WIFI_NOT_INIT");
        break;
      case ESP_ERR_INVALID_ARG:
        Serial.println("ESP_ERR_INVALID_ARG");
        break;
      case ESP_ERR_WIFI_NOT_CONNECT:
        Serial.println("ESP_ERR_WIFI_NOT_CONNECT");
        break;
      default:
        Serial.println(err);
        break;
    }
  }
}

const String reset_reason_to_string(int reason, bool verbose=false) {
  if (verbose) {
    switch (reason) {
      case 1  : return ("Vbat power on reset");
      case 3  : return ("Software reset digital core");
      case 4  : return ("Legacy watch dog reset digital core");
      case 5  : return ("Deep Sleep reset digital core");
      case 6  : return ("Reset by SLC module, reset digital core");
      case 7  : return ("Timer Group0 Watch dog reset digital core");
      case 8  : return ("Timer Group1 Watch dog reset digital core");
      case 9  : return ("RTC Watch dog Reset digital core");
      case 10 : return ("Instrusion tested to reset CPU");
      case 11 : return ("Time Group reset CPU");
      case 12 : return ("Software reset CPU");
      case 13 : return ("RTC Watch dog Reset CPU");
      case 14 : return ("for APP CPU, reseted by PRO CPU");
      case 15 : return ("Reset when the vdd voltage is not stable");
      case 16 : return ("RTC Watch dog reset digital core and rtc module");
      default : return ("NO_MEAN");
    }
  } else {
    switch (reason) {
      case 1 : return ("POWERON_RESET");
      case 3 : return ("SW_RESET");
      case 4 : return ("OWDT_RESET");
      case 5 : return ("DEEPSLEEP_RESET");
      case 6 : return ("SDIO_RESET");
      case 7 : return ("TG0WDT_SYS_RESET");
      case 8 : return ("TG1WDT_SYS_RESET");
      case 9 : return ("RTCWDT_SYS_RESET");
      case 10 : return ("INTRUSION_RESET");
      case 11 : return ("TGWDT_CPU_RESET");
      case 12 : return ("SW_CPU_RESET");
      case 13 : return ("RTCWDT_CPU_RESET");
      case 14 : return ("EXT_CPU_RESET");
      case 15 : return ("RTCWDT_BROWN_OUT_RESET");
      case 16 : return ("RTCWDT_RTC_RESET");
      default : return ("NO_MEAN");
    }
  }
}
