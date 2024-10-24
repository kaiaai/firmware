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

#include <motor_ctl.h>
#include "robot_config.h"
#include "param_file.h"
#include "util.h"

const uint8_t MOTOR_COUNT = 2;
MotorController motorLeft, motorRight;
extern CONFIG cfg;
extern PARAM_FILE params;

enum motor_driver_t {
  MOT_DRIVER_PWM_CW,
  MOT_DRIVER_IN1_IN2, // Generic
};
enum motor_encoder_t {
  MOT_ENCODER_FG,
  MOT_ENCODER_AB_QUAD,
};

motor_driver_t motorDriverType;

void IRAM_ATTR unsignedEncoderLeftISR() {
  motorLeft.tickUnsignedEncoder();
}

void IRAM_ATTR unsignedEncoderRightISR() {
  motorRight.tickUnsignedEncoder();
}

void IRAM_ATTR quadEncoderALeftISR() {
  byte enc_a = digitalRead(cfg.MOT_ENC_A_LEFT_PIN);
  byte enc_b = digitalRead(cfg.MOT_ENC_B_LEFT_PIN);
  motorLeft.tickSignedEncoder(enc_a != enc_b);
}

void IRAM_ATTR quadEncoderARightISR() {
  byte enc_a = digitalRead(cfg.MOT_ENC_A_RIGHT_PIN);
  byte enc_b = digitalRead(cfg.MOT_ENC_B_RIGHT_PIN);
  motorRight.tickSignedEncoder(enc_a != enc_b);
}

void IRAM_ATTR quadEncoderBLeftISR() {
  byte enc_a = digitalRead(cfg.MOT_ENC_A_LEFT_PIN);
  byte enc_b = digitalRead(cfg.MOT_ENC_B_LEFT_PIN);
  motorLeft.tickSignedEncoder(enc_a == enc_b);
}

void IRAM_ATTR quadEncoderBRightISR() {
  byte enc_a = digitalRead(cfg.MOT_ENC_A_RIGHT_PIN);
  byte enc_b = digitalRead(cfg.MOT_ENC_B_RIGHT_PIN);
  motorRight.tickSignedEncoder(enc_a == enc_b);
}

void setMotorPWM(MotorController *motor_controller, float pwm) {
  //printNB("setMotorPWM ");
  bool is_right = motor_controller == &motorRight;
  //printNB(is_right ? " right " : " left ");
  //printlnNB(String(pwm));
  
  uint8_t pwm_channel = is_right ? cfg.MOT_PWM_RIGHT_CHANNEL : cfg.MOT_PWM_LEFT_CHANNEL;
  int max_pwm = (1<<cfg.MOT_PWM_BITS) - 1;
  uint8_t cw_pin = is_right ? cfg.MOT_CW_RIGHT_PIN : cfg.MOT_CW_LEFT_PIN;
  uint8_t in1_pin = is_right ? cfg.MOT_IN1_RIGHT_PIN : cfg.MOT_IN1_LEFT_PIN;
  uint8_t in2_pin = is_right ? cfg.MOT_IN2_RIGHT_PIN : cfg.MOT_IN2_LEFT_PIN;

  pwm = pwm > 1 ? 1 : pwm;
  int pwm_magnitude = round(max_pwm*(1 - abs(pwm)));
  byte cw_value = pwm >= 0 ? LOW : HIGH;

  switch(motorDriverType) {
    case MOT_DRIVER_PWM_CW:
      ledcWrite(pwm_channel, pwm_magnitude);
      digitalWrite (cw_pin, cw_value); //pwm >= 0 ? LOW : HIGH);
      break;

    default:
      if (pwm == 0) {
        // Hard brake
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, HIGH);
        //Serial.println("hard brake");
        return;
      } else if (pwm < -1) {
        // Soft brake
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        //Serial.println("soft brake");
        return;
      }
      
      uint8_t in1 = pwm > 0 ? in1_pin : in2_pin;
      uint8_t in2 = pwm > 0 ? in2_pin : in1_pin;
      
      ledcAttachPin(in2, pwm_channel);
      //ledcAttachChannel(in2, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS, pwm_channel);
      ledcWrite(pwm_channel, pwm_magnitude);
      setPinMode(in1, OUTPUT);
      digitalWrite(in1, HIGH);
    break;
  }
}

void setupEncoders(motor_encoder_t motor_encoder_type) {
  switch(motor_encoder_type) {
    case MOT_ENCODER_AB_QUAD:
      motorLeft.init(MotorController::ENCODER_SIGNED, 4);
      motorRight.init(MotorController::ENCODER_SIGNED, 4);

      setPinMode(cfg.MOT_ENC_A_LEFT_PIN, INPUT);
      setPinMode(cfg.MOT_ENC_B_LEFT_PIN, INPUT);
      attachInterrupt(cfg.MOT_ENC_A_LEFT_PIN, quadEncoderALeftISR, CHANGE);
      attachInterrupt(cfg.MOT_ENC_B_LEFT_PIN, quadEncoderBLeftISR, CHANGE);
    
      setPinMode(cfg.MOT_ENC_A_RIGHT_PIN, INPUT);
      setPinMode(cfg.MOT_ENC_B_RIGHT_PIN, INPUT);
      attachInterrupt(cfg.MOT_ENC_A_RIGHT_PIN, quadEncoderARightISR, CHANGE);
      attachInterrupt(cfg.MOT_ENC_B_RIGHT_PIN, quadEncoderBRightISR, CHANGE);
      break;
    default:
      motorLeft.init(MotorController::ENCODER_UNSIGNED, 2);
      motorRight.init(MotorController::ENCODER_UNSIGNED, 2);
   
      setPinMode(cfg.MOT_FG_LEFT_PIN, INPUT);
      attachInterrupt(cfg.MOT_FG_LEFT_PIN, unsignedEncoderLeftISR, CHANGE);
    
      setPinMode(cfg.MOT_FG_RIGHT_PIN, INPUT);
      attachInterrupt(cfg.MOT_FG_RIGHT_PIN, unsignedEncoderRightISR, CHANGE);
      break;
  }
}

void setupDriver(motor_driver_t motor_driver_type) {
  motorDriverType = motor_driver_type;

  switch(motorDriverType) {
    case MOT_DRIVER_PWM_CW:    
      setPinMode(cfg.MOT_CW_LEFT_PIN, OUTPUT);
      setPinMode(cfg.MOT_CW_RIGHT_PIN, OUTPUT);

      ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      setPinMode(cfg.MOT_PWM_LEFT_PIN, OUTPUT);
      ledcAttachPin(cfg.MOT_PWM_LEFT_PIN, cfg.MOT_PWM_LEFT_CHANNEL);
      //ledcAttachChannel(cfg.MOT_PWM_LEFT_PIN, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS, cfg.MOT_PWM_LEFT_CHANNEL);
    
      ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      setPinMode(cfg.MOT_PWM_RIGHT_PIN, OUTPUT);
      ledcAttachPin(cfg.MOT_PWM_RIGHT_PIN, cfg.MOT_PWM_RIGHT_CHANNEL);
      //ledcAttachChannel(cfg.MOT_PWM_RIGHT_PIN, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS, cfg.MOT_PWM_RIGHT_CHANNEL);
      break;
    default:
      setPinMode(cfg.MOT_IN1_LEFT_PIN, OUTPUT);
      setPinMode(cfg.MOT_IN2_LEFT_PIN, OUTPUT);
      ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      //ledcAttachChannel(cfg.MOT_IN1_LEFT_PIN, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS, cfg.MOT_PWM_LEFT_CHANNEL);

      setPinMode(cfg.MOT_IN1_RIGHT_PIN, OUTPUT);
      setPinMode(cfg.MOT_IN2_RIGHT_PIN, OUTPUT);
      ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      //ledcAttachChannel(cfg.MOT_IN2_RIGHT_PIN, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS, cfg.MOT_PWM_RIGHT_CHANNEL);
      break;
  }
}

void setupMotors() {
  const char * motor_driver_type = params.get(cfg.PARAM_MOTOR_DRIVER_TYPE);
  Serial.print("Motor driver type ");
  Serial.print(motor_driver_type);

  if (strcmp(motor_driver_type, "PWM_CW") == 0) {
    setupDriver(MOT_DRIVER_PWM_CW);
  } else {
    if (strcmp(motor_driver_type, "IN1_IN2") != 0)
      Serial.println(" not recognized, defaulting to IN1_IN2 generic");
    setupDriver(MOT_DRIVER_IN1_IN2);
  }

  motorLeft.setPWMCallback(setMotorPWM);
  motorRight.setPWMCallback(setMotorPWM);

  setMotorPWM(&motorLeft, 0);
  setMotorPWM(&motorRight, 0);

  const char * motor_encoder_type = params.get(cfg.PARAM_MOTOR_ENCODER_TYPE);
  Serial.print("; motor encoder type ");
  Serial.print(motor_encoder_type);

  if (strcmp(motor_encoder_type, "FG") == 0) {
    setupEncoders(MOT_ENCODER_FG);
  } else {
    if (strcmp(motor_encoder_type, "AB_QUAD") != 0)
      Serial.print(" not recognized, defaulting to AB_QUAD");
    setupEncoders(MOT_ENCODER_AB_QUAD);
  }
  Serial.println();

  float value = params.getAsFloat(cfg.PARAM_MOTOR_MAX_RPM);
  Serial.print("Motor Max RPM ");
  Serial.print(value);
  float derate = params.getAsFloat(cfg.PARAM_MOTOR_MAX_RPM_DERATE);
  float max_RPM_derated = value * derate;
  motorLeft.setMaxRPM(max_RPM_derated);
  motorRight.setMaxRPM(max_RPM_derated);
  Serial.print(", derated Max RPM ");
  Serial.print(motorLeft.getMaxRPM());

  value = params.getAsFloat(cfg.PARAM_WHEEL_PPR);
  motorLeft.setEncoderPPR(value);
  motorRight.setEncoderPPR(value);
  Serial.print("; encoder PPR ");
  Serial.print(motorLeft.getEncoderPPR());
  Serial.print(" TPR "); // encoder ticks per revolution
  Serial.println(motorLeft.getEncoderTPR());

  float kp = params.getAsFloat(cfg.PARAM_MOTOR_PID_KP);
  float ki = params.getAsFloat(cfg.PARAM_MOTOR_PID_KI);
  float kd = params.getAsFloat(cfg.PARAM_MOTOR_PID_KD);
  float period = params.getAsFloat(cfg.PARAM_MOTOR_PID_PERIOD);
  const char * pid_mode = params.get(cfg.PARAM_MOTOR_PID_MODE);
  bool on_error = strcmp(pid_mode, "ON_ERROR") == 0;

  motorLeft.setPIDConfig(kp, ki, kd, period, on_error);
  motorRight.setPIDConfig(kp, ki, kd, period, on_error);

  const char * motor_reversed = params.get(cfg.PARAM_MOTOR_DIRECTION_REVERSED);
  bool motor_reversed_left = false;
  bool motor_reversed_right = false;

  if (strcmp(motor_reversed, "LEFT") == 0) {
    motor_reversed_left = true;
  } else if (strcmp(motor_reversed, "RIGHT") == 0) {
    motor_reversed_right = true;
  } else if (strcmp(motor_reversed, "BOTH") == 0) {
    motor_reversed_left = true;
    motor_reversed_right = true;    
  }

  motorLeft.reverseMotor(motor_reversed_left);
  motorRight.reverseMotor(motor_reversed_right);

  Serial.print("Motor direction reversed left ");
  Serial.print(motor_reversed_left);
  Serial.print(", right ");
  Serial.print(motor_reversed_right);

  const char * encoder_reversed = params.get(cfg.PARAM_MOTOR_ENCODER_REVERSED);
  bool encoder_reversed_left = false;
  bool encoder_reversed_right = false;

  if (strcmp(encoder_reversed, "LEFT") == 0) {
    encoder_reversed_left = true;
  } else if (strcmp(encoder_reversed, "RIGHT") == 0) {
    encoder_reversed_right = true;
  } else if (strcmp(encoder_reversed, "BOTH") == 0) {
    encoder_reversed_left = true;
    encoder_reversed_right = true;    
  }

  motorLeft.reverseEncoder(encoder_reversed_left);
  motorRight.reverseEncoder(encoder_reversed_right);

  Serial.print("; encoder reversed left ");
  Serial.print(encoder_reversed_left);
  Serial.print(", right ");
  Serial.println(encoder_reversed_right);
}

void setMotorSpeeds(float rpm_left, float rpm_right) {
  motorRight.setTargetRPM(rpm_right);
  motorLeft.setTargetRPM(rpm_left);
  //Serial.print("setMotorSpeeds ");
  //Serial.print(rpm_right);
  //Serial.print(" ");
  //Serial.println(rpm_left);
}
