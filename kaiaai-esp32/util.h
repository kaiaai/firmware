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

#include <arduino.h>
#include "robot_config.h"

void blink(unsigned int delay_ms, unsigned int count);
float absMin(float a, float b_abs);
void delaySpin(unsigned long msec);
void printCurrentTime();
void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
  float *speed_right, float *speed_left);
void delayYield(unsigned long msec);
const String micro_ros_error_string(int err);

const String reset_reason_to_string(int reason, bool verbose=false);
