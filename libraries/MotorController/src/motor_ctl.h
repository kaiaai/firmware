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

#include <PID_Timed.h>
#include <Arduino.h>

class MotorController {
  public:
    typedef void (*SetPWMCallback)(MotorController*, float);
    enum encoder_type_t {
      ENCODER_UNSIGNED,
      ENCODER_SIGNED,
    };

    void init(encoder_type_t encoder_type, uint8_t ticks_per_pulse);
    void setPWMCallback(SetPWMCallback callback);
    bool setTargetRPM(float rpm);
    void resetEncoders();
    void update();
    float getShaftAngle();
    void reverseMotor(bool reversed);
    void reverseEncoder(bool reversed);
    void setMaxRPM(float rpm);
    void setEncoderPPR(float ppr);
    void setPIDConfig(float kp, float ki, float kd, float period, bool on_error);
    void setPIDKp(float kp);
    void setPIDKi(float ki);
    void setPIDKd(float kd);
    void setPIDPeriod(float period);
    void setPIDOnError(bool on_error);
    float getCurrentPWM();
    float getCurrentRPM();
    float getTargetRPM();
    float getMaxRPM();
    float getEncoderTPR();
    float getEncoderPPR();
    float getPIDKp();
    float getPIDKi();
    float getPIDKd();
    float getPIDPeriod();
    bool getPIDOnError();
    void enablePID(bool en);
    long int getEncoderValue() {
      return encoderReversed ? -encoder : encoder;
    }

  protected:
    volatile long int encoder;
    bool encoderReversed;

    void setPWM(float value);
    SetPWMCallback set_pwm_callback;
    PID_FLOAT pid;
    float pidPWM;
    float targetRPM;
    float measuredRPM;
    float pwm;
    float maxRPM;
    bool cw;

    uint8_t ticksPerPulse;
    float encoderPPR;
    float encoderTPR;
    float encoderTPR_reciprocal;
    float ticksPerMicroSecToRPM;

    unsigned int pidUpdatePeriodUs;
    encoder_type_t encoderType;
    long int encPrev;
    bool setPointHasChanged;
    bool motorReversed;
    unsigned long tickSampleTimePrev;
    bool switchingCw;

  public:
    void tickSignedEncoder(bool increment) {
//      if (increment ^ encoderReversed)
      if (increment)
        encoder = encoder + 1;
      else
        encoder = encoder - 1;
    }
    void tickUnsignedEncoder() {
//      if (cw ^ encoderReversed)
      if (cw)
        encoder = encoder + 1;
      else
        encoder = encoder - 1;
    }
};
