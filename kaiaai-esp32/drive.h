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

#include <math.h>
#include <Arduino.h>
#include <PID_Timed.h>

typedef void (*logFuncT)(char*);

class DriveController {
  public:
    enum motors {
      MOTOR_LEFT = 0,
      MOTOR_RIGHT = 1,
      MOTOR_COUNT = 2,
    };
    static constexpr float DEFAULT_PID_UPDATE_PERIOD = 0.03; // default in seconds
    static constexpr float DEFAULT_PID_KP_WHEEL = 0.001; // default; 0.003 P_ON_E
    static constexpr float DEFAULT_PID_KI_WHEEL = 0.001; // default; 0.0002
    static constexpr float DEFAULT_PID_KD_WHEEL = 0; // default
    static const int8_t DEFAULT_PID_MODE = PID::P_ON_M; // default; P_ON_E
    static const bool FLIP_ROTATION = true;
    static const uint16_t DEFAULT_PWM_FREQ = 20000; // 15..25KHz
    static const uint8_t PWM_BITS = 10;
    static const uint16_t PWM_MAX = (1<<PWM_BITS);  // 1024; TODO 1023?

    // Chihai Motor CHR-GM25-BL2418 24V 200RPM max, 145RPM rated, 45 gear ratio, 6 PPR
    static const uint16_t DEFAULT_MOTOR_MAX_RPM = 200*0.9;  // no-load, derated
    // encoder pulses per gearbox shaft revolution
    // WHEEL_ENCODER_PPR = (MOTOR_GEAR_RATIO*MOTOR_ENCODER_PPR)
    static constexpr float DEFAULT_WHEEL_ENCODER_PPR = 45.0*6;

  public:
    DriveController(uint8_t pwm_left_pin, uint8_t pwm_right_pin,
      uint8_t cw_left_pin, uint8_t cw_right_pin,
      uint8_t fg_left_pin, uint8_t fg_right_pin);
    void initOnce(logFuncT logFunc);
    bool setRPM(unsigned char motorID, float rpm);
    void resetEncoders();
    void update();
    float getShaftAngle(unsigned char motorID);
    void setPIDUpdatePeriod(float period);
    void setPWM(unsigned char motorID, float pwm);
    void enablePID(bool en);
    void setMaxRPM(float rpm);
    void setEncoderPPR(float ppr);

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setProportionalMode(bool onMeasurement);
    void setPWMFreq(unsigned short int freq);

    double getCurrentRPM(unsigned char motorID);
    double getTargetRPM(unsigned char motorID);
    float getCurrentPWM(unsigned char motorID);
    float getMaxRPM();

  private:
    PID *pid[MOTOR_COUNT];
    unsigned int pidUpdatePeriodUs;
    unsigned long tickSampleTimePrev;
    unsigned long tickSampleTimeDelta;

    double targetRPM[MOTOR_COUNT];
    double measuredRPM[MOTOR_COUNT];
    double pidPWM[MOTOR_COUNT];
    int PWM[MOTOR_COUNT];

    double ticksPerMicroSecToRPM;
    long int encDelta[MOTOR_COUNT];
    float maxRPM;
    float encoderTPR;
    long int encPrev[MOTOR_COUNT];
    bool setPointHasChanged[MOTOR_COUNT];

    uint8_t pwmPin[MOTOR_COUNT];
    uint8_t cwPin[MOTOR_COUNT];

    bool switchingCw[MOTOR_COUNT];
    logFuncT logDebug;
};
