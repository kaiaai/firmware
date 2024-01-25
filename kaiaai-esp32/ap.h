// Based on:
//   TODO Arduino library
//
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

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>

class AP {
public:
  typedef bool (*set_param_t)(const char*, const char*);
  void obtainConfig(void (*callback)(), const char * SSID_AP, set_param_t set_param_callback);
protected:
  static constexpr char * CHAR_ENCODING = (char *)"text/html; charset=utf-8";
};
