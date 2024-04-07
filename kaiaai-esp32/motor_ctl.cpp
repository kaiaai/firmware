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

#include "drive_ctl.h"

void MotorController::init() {

  setMaxRPM(200);
  setEncoderPPR(45.0*6);

  pinMode(cwPin[motorID], OUTPUT);
  ledcAttachPin(pwmPin[motorID], motorID);

  targetRPM = 0;
  measuredRPM = 0;
  pidPWM = 0;
  encPrev = 0;
  setPointHasChanged = false;
  tickSampleTimePrev = 0;
  motorReversed = false;
  encoderReversed = false;

  float updatePeriodSec = 0.03;
  pid.Init(&measuredRPM, &pidPWM, &targetRPM, 0.001, 0.001, 0,
    updatePeriodSec, PID::P_ON_M, PID::DIRECT);
  
  pid.SetOutputLimits(-1, 1);

  pwm = 1; // force update

  setPIDUpdatePeriod(updatePeriodSec);
  enablePID(true);
}

void MotorController::setPWMCallback(SetPWMCallback set_pwm_callback) {
  this->set_pwm_callback = set_pwm_callback;
}

void MotorController::setPWM(float value) {
  if (set_pwm_callback)
    set_pwm_callback(value);
}

float MotorController::getShaftAngle() {
//  return TWO_PI * encoder / encoderTPR;
  return TWO_PI * encoder * encoderTPR_reciprocal;
}

void MotorController::setMaxRPM(float rpm) {
  maxRPM = abs(rpm);
}

void MotorController::enablePID(bool en) {
  pid.enable(en);
}

void MotorController::setEncoderPPR(float ppr) {
  if (ppr <= 0)
    return;
  float tpr = 2*ppr; // two edges per pulse
  encoderTPR_reciprocal = 1.0f / tpr;
//  ticksPerMicroSecToRPM = 1e6 * 60.0 / tpr;
  ticksPerMicroSecToRPM = 1e6 * 60.0 * encoderTPR_reciprocal;
}

float MotorController::getMaxRPM() {
  return maxRPM;
}

float MotorController::getCurrentRPM() {
  return measuredRPM;
}

float MotorController::getTargetRPM() {
  return targetRPM;
}

void MotorController::resetEncoders() {
  encoder = 0;
}

void MotorController::setPIDConfig(float kp, float ki, float kd, float period, bool on_error) {
  pidUpdatePeriodUs = (unsigned int) round(period * 1e6);

  pid.SetTunings(kp, ki, kd, on_meas ? PID::P_ON_E : PID::P_ON_M);
}

void MotorController::getPIDConfig(float &kp, float &ki, float &kd, float &period, bool &on_error) {
  kp = pid.GetKp();
  ki = pid.GetKi();
  kd = pid.GetKd();
  period = pidUpdatePeriodUs * 1e-6;
  on_error = !pid.isOnError;
}

bool MotorController::setRPM(float rpm) {
  if (targetRPM == rpm)
    return false;

  bool within_limit = (abs(rpm) <= maxRPM);
  rpm = within_limit ? rpm : maxRPM;

  targetRPM = rpm;
  setPointHasChanged = true;
  return within_limit;
}

void MotorController::setEncoderDirection(bool reverse) {
  encoderReversed = reversed;
}

void MotorController::setMotorDirection(bool reversed) {
  motorReversed = reversed;
}
