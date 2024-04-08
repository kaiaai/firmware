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
    typedef void (*SetPWMCallback)(float);

    virtual void init();
    void setPWMCallback(SetPWMCallback callback);
    bool setRPM(float rpm);
    void resetEncoders();
    virtual void update() = 0;
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

    volatile long int encoder; // 0
    volatile uint8_t encoder_dir; // true

  protected:  
    virtual void setPWM(float value);
    SetPWMCallback set_pwm_callback;
    float encoderTPR;
    PID_FLOAT pid;
    float pidPWM;
    unsigned int pidUpdatePeriodUs;
    float targetRPM;
    float measuredRPM;
    float maxRPM;
    //float encoderTPR;
    float encoderTPR_reciprocal;
    float pwm;

    float ticksPerMicroSecToRPM;
    long int encDelta;
    long int encPrev;
    bool setPointHasChanged;
    bool motorReversed;
    bool encoderReversed;
    unsigned long tickSampleTimePrev;
};
