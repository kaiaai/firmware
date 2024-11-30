#pragma once
#include <Arduino.h>

class PID
{
public:

  PID(); // Call init

  // Constructor. Links the PID to the Input, Output, and 
  // Setpoint. Initial tuning parameters are also set here.
  // (overload for specifying proportional mode)
  PID(double*, double*, double*, double, double, double, double, double);

  void Init(double*, double*, double*, double, double, double, double, double);

  // true Auto, false Manual
  void enable(bool en);

  // Perform the PID calculation. It should be called every time loop() cycles
  bool Compute(double SampleTimeSec);

  // Clamp output to a specific range. 0-255 by default, but
  // it's likely the user will want to change this depending on the application
  void SetOutputLimits(double, double);

  // While most users will set the tunings once in the 
  // constructor, this function gives the user the option
  // of changing tunings during runtime for Adaptive control
  void SetTunings(double, double, double, double);

  // Sets the period, in Milliseconds, with which 
  // the PID calculation is performed. Default is 100.
  // Ki, Kd are referenced to this period
  void SetReferenceSampleTime(double);

  // Zero out integral of error
  void clearErrorIntegral();

  double GetKpe();
  double GetKpm();
  double GetKi();
  double GetKd();
  bool isEnabled(); // Pause, unpause calculations
  double GetReferenceSampleTime();

private:
  void Initialize();
	
  double kpe; // (P)roportional on error Tuning Parameter
  double ki; // (I)ntegral Tuning Parameter
  double kd; // (D)erivative Tuning Parameter
  double kpm; // (P)roportional on measurement Tuning Parameter

  // Pointers to the Input, Output, and Setpoint variables
  //   This creates a hard link between the variables and the 
  //   PID, freeing the user from having to constantly tell us
  //   what these values are.  with pointers we'll just know.
  double *myInput;
  double *myOutput;
  double *mySetpoint;

  double outputSum, lastInput;

  double ReferenceSampleTime;
  double outMin, outMax;
  bool inAuto;
};

class PID_FLOAT
{
public:

  PID_FLOAT(); // Call init

  // Constructor. Links the PID to the Input, Output, and 
  // Setpoint. Initial tuning parameters are also set here.
  // (overload for specifying proportional mode)
  PID_FLOAT(float*, float*, float*, float, float, float, float, float);

  void Init(float*, float*, float*, float, float, float, float, float);

  // true Auto, false Manual
  void enable(bool en);

  // Perform the PID calculation. It should be called every time loop() cycles
  bool Compute(float SampleTimeSec);

  // Clamp output to a specific range. 0-255 by default, but
  // it's likely the user will want to change this depending on the application
  void SetOutputLimits(float, float);

  // While most users will set the tunings once in the 
  // constructor, this function gives the user the option
  // of changing tunings during runtime for Adaptive control
  void SetTunings(float, float, float, float);         	  

  // Set the Direction, or "Action" of the controller. DIRECT
  // means the output will increase when error is positive. REVERSE
  // means the opposite.  it's very unlikely that this will be needed
  // once it is set in the constructor.
  void SetControllerDirection(int);

  // Sets the frequency, in Milliseconds, with which 
  // the PID calculation is performed. Default is 100.
  // Ki, Kd are referenced to this period
  void SetReferenceSampleTime(float);

  // Zero out integral of error
  void clearErrorIntegral();

  float GetKpe();
  float GetKpm();
  float GetKi();
  float GetKd();
  bool isEnabled(); // Pause, unpause calculations
  float GetReferenceSampleTime();

private:
  void Initialize();
	
  float kpe; // (P)roportional on error Tuning Parameter
  float ki; // (I)ntegral Tuning Parameter
  float kd; // (D)erivative Tuning Parameter
  float kpm; // (P)roportional on measurement Tuning Parameter

  // Pointers to the Input, Output, and Setpoint variables
  //   This creates a hard link between the variables and the 
  //   PID, freeing the user from having to constantly tell us
  //   what these values are.  with pointers we'll just know.
  float *myInput;
  float *myOutput;
  float *mySetpoint;

  float outputSum, lastInput;

  float ReferenceSampleTime;
  float outMin, outMax;
  bool inAuto;
};