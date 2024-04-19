# MotorController

- implements PID to convert desired RPM to motor signal (e.g. PWM)
- can be used to drive
  - brushless motors (PWM and CW motor driver pins)
  - brushed motors (IN1, IN2 motor driver pins)
- can be used with
  - quadrature (signed) encoders (ENC_A, ENC_B pins)
  - unsigned encoders (e.g. brushless FG feedback signal)

Requires PID_Timed Arduino library to operate.