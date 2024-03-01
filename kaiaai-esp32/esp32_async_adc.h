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

// Based on
// https://github.com/espressif/arduino-esp32/blob/1.0.4/cores/esp32/esp32-hal-adc.c#L159-L217

#pragma once
#include <Arduino.h>

class ESP32AsyncADC {
protected:
  static const uint8_t __analogWidth = 3; //12 bits
  static const uint8_t __analogReturnedWidth = 12; // Width of returned answer

public:
  static bool IRAM_ATTR adcStart(uint8_t pin);
  static bool IRAM_ATTR adcBusy(uint8_t pin);
  static uint16_t IRAM_ATTR adcEnd(uint8_t pin);
};
