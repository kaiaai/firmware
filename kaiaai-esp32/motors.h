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

#include "motor_ctl.h"
#include "robot_config.h"
#include "param_file.h"

const uint8_t MOTOR_COUNT = 2;
MotorController motorLeft, motorRight;
extern CONFIG cfg;
extern PARAM_FILE params;

enum motor_driver_t {
  MOT_DRIVER_PWM_CW,
  MOT_DRIVER_IN1_IN2_TB6612FNG,
};
enum motor_encoder_t {
  MOT_ENCODER_FG,
  MOT_ENCODER_ENCA_ENCB_QUAD,
};

motor_driver_t motorDriverType;
motor_encoder_t motorEncoderType;

void IRAM_ATTR unsignedEncoderLeftISR() {
  motorLeft.tickUnsignedEncoder();
}

void IRAM_ATTR unsignedEncoderRightISR() {
  motorLeft.tickUnsignedEncoder();
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

void setMotorPWM(MotorController *motor_controller, float pwm) {
  Serial.print("setMotorPWM ");
  bool is_right = motor_controller == &motorRight;
  Serial.print(is_right ? " right " : " left ");
  Serial.println(pwm);
  
  uint8_t pwm_channel = is_right ? cfg.MOT_PWM_RIGHT_CHANNEL : cfg.MOT_PWM_LEFT_CHANNEL;
  int max_pwm = (1<<cfg.MOT_PWM_BITS) - 1;
  int pwm_value;
  uint8_t cw_pin = is_right ? cfg.MOT_CW_RIGHT_PIN : cfg.MOT_CW_LEFT_PIN;
  uint8_t in1_pin = is_right ? cfg.MOT_IN1_RIGHT_PIN : cfg.MOT_IN1_LEFT_PIN;
  uint8_t in2_pin = is_right ? cfg.MOT_IN2_RIGHT_PIN : cfg.MOT_IN2_LEFT_PIN;

  pwm = pwm > 1 ? 1 : pwm;
  pwm_value = round(max_pwm*(1 - abs(pwm)));

  switch(motorDriverType) {
    case MOT_DRIVER_PWM_CW:
      ledcWrite(pwm_channel, pwm_value);
      digitalWrite (cw_pin, pwm ? LOW : HIGH);
      //Serial.print(", MOT_DRIVER_PWM_CW ");
      //Serial.println(pwm_value);
      break;

    default:
      //Serial.print(", MOT_IN1_IN2 ");
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
      ledcWrite(pwm_channel, pwm_value);
      pinMode(in1, OUTPUT);
      digitalWrite(in1, HIGH); 
      //Serial.println(pwm_value);
    break;
  }
}

void setupEncoders(motor_encoder_t motor_encoder) {
  switch(motor_encoder) {
    case MOT_ENCODER_ENCA_ENCB_QUAD:
      motorLeft.init(MotorController::ENCODER_SIGNED);
      motorRight.init(MotorController::ENCODER_SIGNED);

      pinMode(cfg.MOT_ENC_A_LEFT_PIN, INPUT);
      pinMode(cfg.MOT_ENC_B_LEFT_PIN, INPUT);
      attachInterrupt(cfg.MOT_ENC_A_LEFT_PIN, quadEncoderALeftISR, CHANGE);
    
      pinMode(cfg.MOT_ENC_A_RIGHT_PIN, INPUT);
      pinMode(cfg.MOT_ENC_B_RIGHT_PIN, INPUT);
      attachInterrupt(cfg.MOT_ENC_A_RIGHT_PIN, quadEncoderARightISR, CHANGE);
      break;
    default:
      motorLeft.init(MotorController::ENCODER_UNSIGNED);
      motorRight.init(MotorController::ENCODER_UNSIGNED);
   
      pinMode(cfg.MOT_FG_LEFT_PIN, INPUT);
      attachInterrupt(cfg.MOT_FG_LEFT_PIN, unsignedEncoderLeftISR, CHANGE);
    
      pinMode(cfg.MOT_FG_RIGHT_PIN, INPUT);
      attachInterrupt(cfg.MOT_FG_RIGHT_PIN, unsignedEncoderRightISR, CHANGE);
      break;
  }
}

void setupDriver(motor_driver_t motor_driver) {
  motorDriverType = motor_driver;

  switch(motorDriverType) {
    case MOT_DRIVER_PWM_CW:
      pinMode(cfg.MOT_CW_LEFT_PIN, OUTPUT);
      pinMode(cfg.MOT_CW_RIGHT_PIN, OUTPUT);
    
      ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      ledcAttachPin(cfg.MOT_PWM_LEFT_PIN, cfg.MOT_PWM_LEFT_CHANNEL);
    
      ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      ledcAttachPin(cfg.MOT_PWM_RIGHT_PIN, cfg.MOT_PWM_RIGHT_CHANNEL);
      break;
    default:
      pinMode(cfg.MOT_IN1_LEFT_PIN, OUTPUT);
      pinMode(cfg.MOT_IN2_LEFT_PIN, OUTPUT);
      ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
    
      pinMode(cfg.MOT_IN1_RIGHT_PIN, OUTPUT);
      pinMode(cfg.MOT_IN2_RIGHT_PIN, OUTPUT);
      ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS);
      break;
  }
}

void setupMotors() {
  const char * motor_driver = params.get(cfg.PARAM_MOTOR_DRIVER);
  Serial.print("Motor driver type ");
  Serial.print(motor_driver);

  if (strcmp(motor_driver, "PWM_CW") == 0) {
    setupDriver(MOT_DRIVER_PWM_CW);
  } else {
    if (strcmp(motor_driver, "IN1_IN2_TB6612FNG") != 0)
      Serial.print(" not recognized, defaulting to IN1_IN2_TB6612FNG");
    setupDriver(MOT_DRIVER_IN1_IN2_TB6612FNG);
  }
  Serial.println();
  
  const char * motor_encoder = params.get(cfg.PARAM_MOTOR_ENCODER);
  Serial.print("Motor encoder type ");
  Serial.print(motor_encoder);

  if (strcmp(motor_encoder, "FG") == 0) {
    setupEncoders(MOT_ENCODER_FG);
  } else {
    if (strcmp(motor_encoder, "ENCA_ENCB_QUAD") != 0)
      Serial.print(" not recognized, defaulting to ENCA_ENCB_QUAD");
    setupEncoders(MOT_ENCODER_ENCA_ENCB_QUAD);
  }
  Serial.println();

  float value = String(params.get(cfg.PARAM_MOTOR_MAX_RPM)).toFloat();
  Serial.print("Motor Max RPM ");
  Serial.print(value);
  value = value * cfg.MOTOR_MAX_RPM_DERATE;
  motorLeft.setMaxRPM(value);
  motorRight.setMaxRPM(value);

  value = String(params.get(cfg.PARAM_WHEEL_PPR)).toFloat();
  motorLeft.setEncoderPPR(value);
  motorRight.setEncoderPPR(value);
  Serial.print(", encoder PPR ");
  Serial.println(value);

  //motorLeft.setPIDConfig(0.001, 0.001, 0, 0.03, false);
  //motorRight.setPIDConfig(0.001, 0.001, 0, 0.03, false);

  //motorLeft.setMotorDirection(true);
  //motorRight.setMotorDirection(true);

  //motorLeft.setEncoderDirection(true);
  //motorRight.setEncoderDirection(true);

  motorLeft.setPWMCallback(setMotorPWM);
  motorRight.setPWMCallback(setMotorPWM);

  setMotorPWM(&motorLeft, 0);
  setMotorPWM(&motorRight, 0);
}

void setMotorSpeeds(float rpm_left, float rpm_right) {
  motorRight.setRPM(rpm_right);
  motorLeft.setRPM(rpm_left);
//  Serial.print("setMotorSpeeds ");
//  Serial.print(rpm_right);
//  Serial.print(" ");
//  Serial.println(rpm_left);
}
