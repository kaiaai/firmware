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

#include "motor_ctl.h"

void MotorController::init(encoder_type_t encoder_type, uint8_t ticks_per_pulse) {

  encoderType = encoder_type;
  resetEncoders();
  setMaxRPM(200);
  ticksPerPulse = ticks_per_pulse;
  setEncoderPPR(45.0*6);

  targetRPM = 0;
  measuredRPM = 0;
  pidPWM = 0;
  encPrev = 0;
  setPointHasChanged = false;
  tickSampleTimePrev = 0;
  motorReversed = false;
  encoderReversed = false;
  cw = true;

  float updatePeriodSec = 0.03;
  pid.Init(&measuredRPM, &pidPWM, &targetRPM, 0.001, 0.001, 0,
    updatePeriodSec, PID::P_ON_M, PID::DIRECT);

  pidUpdatePeriodUs = (unsigned int) round(updatePeriodSec * 1e6);
  pid.SetOutputLimits(-1, 1);

  pwm = 1.1; // force update
  enablePID(true);
}

void MotorController::setPWMCallback(SetPWMCallback set_pwm_callback) {
  this->set_pwm_callback = set_pwm_callback;
}

void MotorController::setPWM(float value) {
  if ((encoderType == ENCODER_UNSIGNED) && switchingCw)
    return;

  if (value == pwm)
    return;

  bool cw_new = (pwm >= 0);
  
  if ((encoderType == ENCODER_UNSIGNED) &&
   ((pwm > 0 && value < 0) || (pwm < 0 && value > 0))) {
    // when cw/ccw changes, stop pwm:=0, verify 0 enc pulses
    // then flip encoder inc/dec, proceed
    switchingCw = true;
    value = 0;
    cw_new = cw; // keep cw unchanged for encoders until we stop
  }
  
  if (set_pwm_callback)
    set_pwm_callback(this, value);

  pwm = value;
  cw = cw_new;
}

float MotorController::getShaftAngle() {
//  return TWO_PI * encoder / encoderTPR;
  return TWO_PI * getEncoderValue() * encoderTPR_reciprocal;
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
  encoderTPR = ticksPerPulse*ppr;
  encoderTPR_reciprocal = 1.0f / encoderTPR;
//  ticksPerMicroSecToRPM = 1e6 * 60.0 / tpr;
  ticksPerMicroSecToRPM = 1e6 * 60.0 * encoderTPR_reciprocal;
}

float MotorController::getEncoderTPR() {
  return encoderTPR;
}

float MotorController::getMaxRPM() {
  return maxRPM;
}

float MotorController::getCurrentRPM() {
  return motorReversed ? -measuredRPM : measuredRPM;
}

float MotorController::getTargetRPM() {
  return motorReversed ? -targetRPM : targetRPM;
}

void MotorController::resetEncoders() {
  encoder = 0;
  switchingCw = false;
}

void MotorController::setPIDConfig(float kp, float ki, float kd, float period, bool on_error) {
  pidUpdatePeriodUs = (unsigned int) round(period * 1e6);
  pid.SetTunings(kp, ki, kd, on_error ? PID::P_ON_E : PID::P_ON_M);
}

void MotorController::getPIDConfig(float &kp, float &ki, float &kd, float &period, bool &on_error) {
  kp = pid.GetKp();
  ki = pid.GetKi();
  kd = pid.GetKd();
  period = pidUpdatePeriodUs * 1e-6;
  on_error = !pid.isOnError();
}

bool MotorController::setRPM(float rpm) {
  rpm = motorReversed ? -rpm : rpm;
  if (targetRPM == rpm)
    return false;

  bool within_limit = (abs(rpm) <= maxRPM);
  rpm = within_limit ? rpm : maxRPM;

  targetRPM = rpm;
  setPointHasChanged = true;
  return within_limit;
}

void MotorController::reverseEncoder(bool reversed) {
  encoderReversed = reversed;
}

void MotorController::reverseMotor(bool reversed) {
  motorReversed = reversed;
}

// TODO detect stuck (stalled) motor, limit current, let robot know
// NB BLDC motor has a built-in feature that shuts motor off after stall timeout
void MotorController::update() {  
  unsigned long tickTime = micros();
  unsigned long tickTimeDelta = tickTime - tickSampleTimePrev;
  if ((tickTimeDelta < pidUpdatePeriodUs) && !setPointHasChanged)
    return;

  tickSampleTimePrev = tickTime;

  long int encNow = getEncoderValue();
  long int encDelta = encNow - encPrev;
  encPrev = encNow;
  float ticksPerMicroSec = ((float) encDelta) / ((float) tickTimeDelta);
  measuredRPM = ticksPerMicroSec * ticksPerMicroSecToRPM;

  setPointHasChanged = false;

  if ((encoderType == ENCODER_UNSIGNED) && (encDelta == 0))
    switchingCw = false;

  if (targetRPM == 0 && measuredRPM == 0) {
      // Prevent wheels from twitching or slowly turning after stop
      pid.clearErrorIntegral();
  }
  
  float sampleTime = tickTimeDelta * 1e-6;
  pid.Compute(sampleTime);
  setPWM(pidPWM);
}
