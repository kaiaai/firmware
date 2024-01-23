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

#include "drive.h"

#define DEBUG true
volatile long int encoder[DriveController::MOTOR_COUNT] = {0, 0};
volatile bool CW[DriveController::MOTOR_COUNT] = {true, true};

void IRAM_ATTR encLeftIsr() {
  if (CW[DriveController::MOTOR_LEFT] ^ DriveController::FLIP_ROTATION)
    encoder[DriveController::MOTOR_LEFT]++;
  else
    encoder[DriveController::MOTOR_LEFT]--;
}

void IRAM_ATTR encRightIsr() {
  if (CW[DriveController::MOTOR_RIGHT] ^ DriveController::FLIP_ROTATION)
    encoder[DriveController::MOTOR_RIGHT]++;
  else
    encoder[DriveController::MOTOR_RIGHT]--;
}

float DriveController::getShaftAngle(unsigned char motorID) {
  if (motorID < MOTOR_COUNT)
    return TWO_PI * encoder[motorID] / encoderTPR;
  return 0;
}

void DriveController::setPIDUpdatePeriod(float period) {
  pidUpdatePeriodUs = (unsigned int) round(period * 1e6);
  //  pid[motorID]->SetReferenceSampleTime(period);  // unnecessary in PID library
}

void DriveController::enablePID(bool en) {
  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++)
    pid[motorID]->enable(en);
}

void DriveController::setMaxRPM(float rpm) {
  if (rpm <= 0)
    return;
  maxRPM = rpm;
}

void DriveController::setEncoderPPR(float ppr) {
  if (ppr <= 0)
    return;
  float tpr = 2*ppr; // two edges per pulse
  encoderTPR = tpr;
  ticksPerMicroSecToRPM = 1e6 * 60.0 / tpr;
}

void DriveController::setKp(float kp) {
  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    PID * pid_ = pid[motorID];
    pid_->SetTunings(kp, pid_->GetKi(), pid_->GetKd());
  }
}

void DriveController::setKi(float ki) {
  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    PID * pid_ = pid[motorID];
    pid_->SetTunings(pid_->GetKp(), ki, pid_->GetKd());
  }
}

void DriveController::setKd(float kd) {
  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    PID * pid_ = pid[motorID];
    pid_->SetTunings(pid_->GetKp(), pid_->GetKi(), kd);
  }
}

void DriveController::setProportionalMode(bool onMeasurement) {
  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++) {    
    PID * pid_ = pid[motorID];
    pid_->SetTunings(pid_->GetKp(), pid_->GetKi(), pid_->GetKd(),
      onMeasurement ? PID::P_ON_M : PID::P_ON_E);
  }
}

void DriveController::initOnce(logFuncT logFunc) {
  logDebug = logFunc;
  tickSampleTimePrev = 0;

  setMaxRPM(DEFAULT_MOTOR_MAX_RPM);
  setEncoderPPR(DEFAULT_WHEEL_ENCODER_PPR);
  setPWMFreq(DEFAULT_PWM_FREQ);

  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    pinMode(cwPin[motorID], OUTPUT);
    ledcAttachPin(pwmPin[motorID], motorID);

    targetRPM[motorID] = 0;
    measuredRPM[motorID] = 0;
    pidPWM[motorID] = 0;
    encPrev[motorID] = 0;
    //brakingEnabled[motorID] = false;
    setPointHasChanged[motorID] = false;
    switchingCw[motorID] = false;
    
    // https://playground.arduino.cc/Code/PIDLibrary/
    pid[motorID] = new PID(&measuredRPM[motorID], &pidPWM[motorID], &targetRPM[motorID],
      DEFAULT_PID_KP_WHEEL, DEFAULT_PID_KI_WHEEL, DEFAULT_PID_KD_WHEEL,
      DEFAULT_PID_UPDATE_PERIOD, DEFAULT_PID_MODE, PID::DIRECT);
    pid[motorID]->SetOutputLimits(-1, 1);

    PWM[motorID] = 1; // force update
    setPWM(motorID, 0);
  }

  setPIDUpdatePeriod(DEFAULT_PID_UPDATE_PERIOD);
  enablePID(true);
}

bool DriveController::setRPM(unsigned char motorID, float rpm) {
  if (motorID >= MOTOR_COUNT) // TODO print log error
    return false;

  if (targetRPM[motorID] == rpm || rpm < 0)
    return false;

  bool within_limit = (rpm <= maxRPM);
  rpm = within_limit ? rpm : maxRPM;

  targetRPM[motorID] = rpm;
  setPointHasChanged[motorID] = true;
  return within_limit;
}

void DriveController::setPWMFreq(uint16_t freq) {
  for (uint8_t motorID = 0; motorID < MOTOR_COUNT; motorID++)
    // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
    //ledcSetup(pwmMotChannel[motorID], freq, PWM_BITS);
    ledcSetup(motorID, freq, PWM_BITS);
}

void DriveController::setPWM(unsigned char motorID, float value) {
  if (motorID >= MOTOR_COUNT)
    return;

  if (switchingCw[motorID])
    return;
  
  value = min(value, 1.0f);  // clamp to range
  value = max(value, -1.0f);
  
  int pwm = (int) round(value * PWM_MAX);

  if (pwm == PWM[motorID])
    return;

  bool cw = (pwm >= 0) ^ FLIP_ROTATION;

  int prevPWM = PWM[motorID];
  if ((prevPWM > 0 && pwm < 0) || (prevPWM < 0 && pwm > 0)) {
    // when cw/ccw changes, stop pwm:=0, verify 0 enc pulses
    // then flip encoder inc/dec, proceed
    switchingCw[motorID] = true;
    pwm = 0;
    cw = CW[motorID]; // freeze cw for encoders until we stop
  }

  int pwmValue = PWM_MAX - abs(pwm); //(pwm >= 0 ? pwm : -pwm);
  ledcWrite(motorID, pwmValue);
  digitalWrite (cwPin[motorID], cw ? LOW : HIGH);

  CW[motorID] = cw;
  PWM[motorID] = pwm;

  //char logMsg[100];
  //sprintf(logMsg, "Motor %d PWM %d CW %d", motorID, pwm, cw);
  //logDebug(logMsg);
}

void DriveController::resetEncoders() {
  for (unsigned char motorID = 0; motorID < MOTOR_COUNT; motorID++)
    encoder[motorID] = 0;
}

DriveController::DriveController(uint8_t pwm_left_pin,
  uint8_t pwm_right_pin, uint8_t cw_left_pin, uint8_t cw_right_pin,
  uint8_t fg_left_pin, uint8_t fg_right_pin) {
  pwmPin[MOTOR_LEFT] = pwm_left_pin;
  pwmPin[MOTOR_RIGHT] = pwm_right_pin;

  cwPin[MOTOR_LEFT] = cw_left_pin;
  cwPin[MOTOR_RIGHT] = cw_right_pin;

  // Encoders
  pinMode(fg_left_pin, INPUT); // INPUT_PULLUP
  attachInterrupt(fg_left_pin, encLeftIsr, CHANGE);

  pinMode(fg_right_pin, INPUT);
  attachInterrupt(fg_right_pin, encRightIsr, CHANGE);
}

// TODO detect stuck (stalled) motor, limit current, let robot know
void DriveController::update() {
  bool anySetPointHasChanged = false;
  for (unsigned char motorID = 0; motorID < MOTOR_COUNT; motorID++)
    anySetPointHasChanged |= setPointHasChanged[motorID];
  
  unsigned long tickTime = esp_timer_get_time();
  unsigned long tickTimeDelta = tickTime - tickSampleTimePrev;
  if ((tickTimeDelta < pidUpdatePeriodUs) && !anySetPointHasChanged)
    return;

  tickSampleTimePrev = tickTime;
  tickSampleTimeDelta = tickTimeDelta;
  
  // TODO fix threads-and-iterrupts race condition
  // TODO move into ESP32 timer? or disable interrupts?
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
  unsigned char motorID;
  for (motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    long int encNow = encoder[motorID];
    encDelta[motorID] = encNow - encPrev[motorID];    
    encPrev[motorID] = encNow;
    double ticksPerMicroSec = ((double) encDelta[motorID]) / ((double) tickSampleTimeDelta);
    measuredRPM[motorID] = ticksPerMicroSec * ticksPerMicroSecToRPM;

//    if (brakingEnabled[motorID] && targetRPM[motorID] == 0)

// TODO maybe uncomment
//    if (targetRPM[motorID] == 0)
//        pid[motorID]->clearErrorIntegral();
    setPointHasChanged[motorID] = false;

    if (encDelta[motorID] == 0)
      switchingCw[motorID] = false;
  }

  if (targetRPM[MOTOR_LEFT] == 0 && measuredRPM[MOTOR_LEFT] == 0 &&
      targetRPM[MOTOR_RIGHT] == 0 && measuredRPM[MOTOR_RIGHT] == 0) {
      
      // Prevent wheels from twitching or slowly turning after stop
      pid[MOTOR_LEFT]->clearErrorIntegral();
      pid[MOTOR_RIGHT]->clearErrorIntegral();
  }
  
  double sampleTime = tickSampleTimeDelta * 1e-6;
  for (motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    pid[motorID]->Compute(sampleTime);

    // Brushless motor hard-brakes when PWM == 0
//    if (brakingEnabled[motorID] && targetRPM[motorID] == 0)

// TODO maybe uncomment
//    if (targetRPM[motorID] == 0)
//        pidPWM[motorID] = 0;

    setPWM(motorID, (float) pidPWM[motorID]);
  }
}

double DriveController::getCurrentRPM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return measuredRPM[motorID];
}

double DriveController::getTargetRPM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return targetRPM[motorID];
}

float DriveController::getCurrentPWM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return float(PWM[motorID]) / PWM_MAX;
}

float DriveController::getMaxRPM() {
  return maxRPM;
}
