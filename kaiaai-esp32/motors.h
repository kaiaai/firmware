// Copyright 2023-2025 KAIA.AI
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

#include <motor_ctl_kaia.h>
#include "robot_config.h"
#include "util.h"

const uint8_t MOTOR_COUNT = 2;
MotorController motorLeft, motorRight;
extern CONFIG cfg;

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
  byte enc_a = digitalRead(cfg.mot_left_enc_gpio_a_fg);
  byte enc_b = digitalRead(cfg.mot_left_enc_gpio_b);
  motorLeft.tickSignedEncoder(enc_a != enc_b);
}

void IRAM_ATTR quadEncoderARightISR() {
  byte enc_a = digitalRead(cfg.mot_right_enc_gpio_a_fg);
  byte enc_b = digitalRead(cfg.mot_right_enc_gpio_b);
  motorRight.tickSignedEncoder(enc_a != enc_b);
}

void IRAM_ATTR quadEncoderBLeftISR() {
  byte enc_a = digitalRead(cfg.mot_left_enc_gpio_a_fg);
  byte enc_b = digitalRead(cfg.mot_left_enc_gpio_b);
  motorLeft.tickSignedEncoder(enc_a == enc_b);
}

void IRAM_ATTR quadEncoderBRightISR() {
  byte enc_a = digitalRead(cfg.mot_right_enc_gpio_a_fg);
  byte enc_b = digitalRead(cfg.mot_right_enc_gpio_b);
  motorRight.tickSignedEncoder(enc_a == enc_b);
}

void setMotorPWM(MotorController *motor_controller, float pwm) {
  bool is_right = motor_controller == &motorRight;

  //printNB("setMotorPWM ");
  //printNB(is_right ? " right " : " left ");
  //printlnNB(String(pwm));
  
  uint8_t pwm_channel = is_right ? cfg.MOT_PWM_RIGHT_CHANNEL : cfg.MOT_PWM_LEFT_CHANNEL;
  int max_pwm = (1<<cfg.MOT_PWM_BITS) - 1;
  uint8_t cw_pin = is_right ? cfg.mot_right_drv_gpio_in2_cw : cfg.mot_left_drv_gpio_in2_cw;
  uint8_t in1_pin = is_right ? cfg.mot_right_drv_gpio_in1_pwm : cfg.mot_left_drv_gpio_in1_pwm;
  uint8_t in2_pin = is_right ? cfg.mot_right_drv_gpio_in2_cw : cfg.mot_left_drv_gpio_in2_cw;

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
      
      #if ESP_IDF_VERSION_MAJOR >= 5
      if (!ledcAttachChannel(in2, cfg.MOT_PWM_FREQ,
        cfg.MOT_PWM_BITS, pwm_channel))
        Serial.println("setMotorPWM() ledcAttachChannel() error");
      #else
      ledcAttachPin(in2, pwm_channel);
      #endif

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

      setPinMode(cfg.mot_left_enc_gpio_a_fg, INPUT);
      setPinMode(cfg.mot_left_enc_gpio_b, INPUT);
      attachInterrupt(cfg.mot_left_enc_gpio_a_fg, quadEncoderALeftISR, CHANGE);
      attachInterrupt(cfg.mot_left_enc_gpio_b, quadEncoderBLeftISR, CHANGE);
    
      setPinMode(cfg.mot_right_enc_gpio_a_fg, INPUT);
      setPinMode(cfg.mot_right_enc_gpio_b, INPUT);
      attachInterrupt(cfg.mot_right_enc_gpio_a_fg, quadEncoderARightISR, CHANGE);
      attachInterrupt(cfg.mot_right_enc_gpio_b, quadEncoderBRightISR, CHANGE);
      break;
    default:
      motorLeft.init(MotorController::ENCODER_UNSIGNED, 2);
      motorRight.init(MotorController::ENCODER_UNSIGNED, 2);
   
      setPinMode(cfg.mot_left_enc_gpio_a_fg, INPUT);
      attachInterrupt(cfg.mot_left_enc_gpio_a_fg, unsignedEncoderLeftISR, CHANGE);
    
      setPinMode(cfg.mot_right_enc_gpio_a_fg, INPUT);
      attachInterrupt(cfg.mot_right_enc_gpio_a_fg, unsignedEncoderRightISR, CHANGE);
      break;
  }
}

void setupDriver(motor_driver_t motor_driver_type) {
  motorDriverType = motor_driver_type;

  switch(motorDriverType) {
    case MOT_DRIVER_PWM_CW:
      setPinMode(cfg.mot_left_drv_gpio_in2_cw, OUTPUT);
      setPinMode(cfg.mot_right_drv_gpio_in2_cw, OUTPUT);
      setPinMode(cfg.mot_left_drv_gpio_in1_pwm, OUTPUT);
      setPinMode(cfg.mot_right_drv_gpio_in1_pwm, OUTPUT);

      #if ESP_IDF_VERSION_MAJOR >= 5
      if (!ledcAttachChannel(cfg.mot_left_drv_gpio_in1_pwm, cfg.MOT_PWM_FREQ,
             cfg.MOT_PWM_BITS, cfg.MOT_PWM_LEFT_CHANNEL) ||
          !ledcAttachChannel(cfg.mot_right_drv_gpio_in1_pwm, cfg.MOT_PWM_FREQ,
             cfg.MOT_PWM_BITS, cfg.MOT_PWM_RIGHT_CHANNEL))
        Serial.println("setupDriver() ledcAttachChannel() error");
      #else
      if (!ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS) ||
          !ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS))
        Serial.println("setupDriver() ledcSetup() error");
      ledcAttachPin(cfg.mot_left_drv_gpio_in1_pwm, cfg.MOT_PWM_LEFT_CHANNEL);
      ledcAttachPin(cfg.mot_right_drv_gpio_in1_pwm, cfg.MOT_PWM_RIGHT_CHANNEL);
      #endif
    
      break;
    default:
      setPinMode(cfg.mot_left_drv_gpio_in1_pwm, OUTPUT);
      setPinMode(cfg.mot_left_drv_gpio_in2_cw, OUTPUT);
      setPinMode(cfg.mot_right_drv_gpio_in1_pwm, OUTPUT);
      setPinMode(cfg.mot_right_drv_gpio_in2_cw, OUTPUT);

      #if ESP_IDF_VERSION_MAJOR >= 5
      if (!ledcAttachChannel(cfg.mot_left_drv_gpio_in1_pwm, cfg.MOT_PWM_FREQ,
             cfg.MOT_PWM_BITS, cfg.MOT_PWM_LEFT_CHANNEL) ||
          !ledcAttachChannel(cfg.mot_right_drv_gpio_in2_cw, cfg.MOT_PWM_FREQ,
             cfg.MOT_PWM_BITS, cfg.MOT_PWM_RIGHT_CHANNEL))
        Serial.println("setupDriver() ledcAttachChannel() error");            
      #else
      if (!ledcSetup(cfg.MOT_PWM_LEFT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS) ||
          !ledcSetup(cfg.MOT_PWM_RIGHT_CHANNEL, cfg.MOT_PWM_FREQ, cfg.MOT_PWM_BITS))
        Serial.println("setupDriver() ledcSetup() error");            
      #endif

      break;
  }
}

void setupMotors() {
  Serial.print("Motor driver type ");
  Serial.print(cfg.motor_driver_type);

  if (cfg.motor_driver_type == "PWM_CW") {
    setupDriver(MOT_DRIVER_PWM_CW);
  } else {
    if (cfg.motor_driver_type != "IN1_IN2")
      Serial.print(" not recognized, defaulting to IN1_IN2");
    setupDriver(MOT_DRIVER_IN1_IN2);
  }

  motorLeft.setPWMCallback(setMotorPWM);
  motorRight.setPWMCallback(setMotorPWM);

  setMotorPWM(&motorLeft, 0);
  setMotorPWM(&motorRight, 0);

  Serial.print("; motor encoder type ");
  Serial.print(cfg.motor_encoder_type);

  if (cfg.motor_encoder_type == "FG") {
    setupEncoders(MOT_ENCODER_FG);
  } else {
    if (cfg.motor_encoder_type != "AB_QUAD")
      Serial.print(" not recognized, defaulting to AB_QUAD");
    setupEncoders(MOT_ENCODER_AB_QUAD);
  }
  Serial.println();

  motorLeft.setMaxRPM(cfg.motor_rpm_max);
  motorRight.setMaxRPM(cfg.motor_rpm_max);
  Serial.print("Motor Max RPM ");
  Serial.print(motorLeft.getMaxRPM());

  motorLeft.setEncoderPPR(cfg.motor_encoder_ppr);
  motorRight.setEncoderPPR(cfg.motor_encoder_ppr);
  Serial.print("; encoder PPR ");
  Serial.print(motorLeft.getEncoderPPR());
  Serial.print(" TPR "); // ticks per revolution
  Serial.println(motorLeft.getEncoderTPR());

  motorLeft.setPIDConfig(cfg.motor_driver_pid_kp, cfg.motor_driver_pid_ki,
    cfg.motor_driver_pid_kd, cfg.motor_driver_pid_period, cfg.motor_driver_pid_kpm);
  motorRight.setPIDConfig(cfg.motor_driver_pid_kp, cfg.motor_driver_pid_ki,
    cfg.motor_driver_pid_kd, cfg.motor_driver_pid_period, cfg.motor_driver_pid_kpm);

  motorLeft.reverseMotor(cfg.mot_left_drv_reverse);
  motorRight.reverseMotor(cfg.mot_right_drv_reverse);

  bool mot_reversed = cfg.mot_left_drv_reverse || cfg.mot_right_drv_reverse;
  bool enc_reversed = cfg.mot_left_enc_reverse || cfg.mot_right_enc_reverse;
  motorLeft.reverseEncoder(cfg.mot_left_enc_reverse);
  motorRight.reverseEncoder(cfg.mot_right_enc_reverse);

  if (mot_reversed)
    Serial.print("Motor direction reversed: ");

  if (cfg.mot_left_drv_reverse)
    Serial.print("left ");
  if (cfg.mot_left_drv_reverse)
    Serial.print("right");

  if (mot_reversed) {
    Serial.print('.');
    if (enc_reversed)
      Serial.print(' ');
  }

  if (enc_reversed)
    Serial.print("Encoder reversed: ");
  if (cfg.mot_left_enc_reverse)
    Serial.print("left ");
  if (cfg.mot_right_enc_reverse)
    Serial.print("right");

  if (mot_reversed || enc_reversed)
    Serial.println();
}

void setMotorSpeeds(float rpm_left, float rpm_right) {
  motorRight.setTargetRPM(rpm_right);
  motorLeft.setTargetRPM(rpm_left);
  //Serial.print("setMotorSpeeds ");
  //Serial.print(rpm_right);
  //Serial.print(" ");
  //Serial.println(rpm_left);
}
