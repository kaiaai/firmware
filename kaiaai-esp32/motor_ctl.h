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

    void init(encoder_type_t encoder_type);
    void setPWMCallback(SetPWMCallback callback);
    bool setRPM(float rpm);
    void resetEncoders();
    void update();
    float getShaftAngle();
    void setMotorDirection(bool reversed);
    void setEncoderDirection(bool reversed);
    void setMaxRPM(float rpm);
    void setEncoderPPR(float ppr);
    void setPIDConfig(float kp, float ki, float kd, float period, bool on_error);
    void getPIDConfig(float &kp, float &ki, float &kd, float &period, bool &on_error);
    float getCurrentRPM();
    float getTargetRPM();
    float getMaxRPM();
    void enablePID(bool en);

  protected:
    volatile long int encoder;
    bool encoder_reversed;

    void setPWM(float value);
    SetPWMCallback set_pwm_callback;
    PID_FLOAT pid;
    float pidPWM;
    float targetRPM;
    float measuredRPM;
    float pwm;
    bool cw;

    //float encoderTPR;
    float maxRPM;
    float encoderTPR;
    unsigned int pidUpdatePeriodUs;
    float encoderTPR_reciprocal;
    encoder_type_t encoderType;
    float ticksPerMicroSecToRPM;

    long int encDelta;
    long int encPrev;
    bool setPointHasChanged;
    bool motorReversed;
    bool encoderReversed;
    unsigned long tickSampleTimePrev;
    bool switchingCw;

  public:
    void tickSignedEncoder(bool increment) {
      if (increment ^ encoder_reversed)
        encoder++;
      else
        encoder--;      
    }
    void tickUnsignedEncoder() {
      if (cw ^ encoder_reversed)
        encoder++;
      else
        encoder--;
    }
};
