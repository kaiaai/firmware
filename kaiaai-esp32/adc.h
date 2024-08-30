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
#include "robot_config.h"
#include "param_file.h"

extern CONFIG cfg;
extern PARAM_FILE params;
float bat_adc_multiplier = 1;

float getBatteryMilliVolts() {
  float voltage_mv = analogReadMilliVolts(cfg.BAT_ADC_PIN);
  voltage_mv *= bat_adc_multiplier;
  if (voltage_mv < cfg.BAT_PRESENT_MV_MIN)
    voltage_mv = 0;
  return voltage_mv;
}

void setupADC() {
//  if (!adcAttachPin(cfg.BAT_ADC_PIN))
//    Serial.println("adcAttachPin() FAILED");

  bat_adc_multiplier = params.getAsFloat(cfg.PARAM_BATTERY_ADC_ATTENUATION);
  Serial.print("Battery ADC attenuation ");
  Serial.print(bat_adc_multiplier);

  float batt_mv = getBatteryMilliVolts();
  if (batt_mv < cfg.BAT_PRESENT_MV_MIN) {
    Serial.println();
    Serial.println("Battery NOT detected. Check battery switch, "
      "connection or replace battery");
  } else {
    Serial.print(", voltage ");
    Serial.print(batt_mv*0.001f);
    Serial.println("V");
  }
}
