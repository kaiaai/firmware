#pragma once
#include <Arduino.h>

class PID
{
public:

	static const int8_t DIRECT = 0;
	static const int8_t REVERSE = 1;
	static const int8_t P_ON_M = 0;
	static const int8_t P_ON_E = 1;

  PID(); // Call init

  // Constructor. Links the PID to the Input, Output, and 
  // Setpoint. Initial tuning parameters are also set here.
  // (overload for specifying proportional mode)
  PID(double*, double*, double*, double, double, double, double, int, int);

  // Constructor. Links the PID to the Input, Output, and
  // Setpoint. Initial tuning parameters are also set here.
  PID(double*, double*, double*, double, double, double, double, int);

  void Init(double*, double*, double*, double, double, double, double, int, int);

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
  void SetTunings(double, double, double);

  // Overload for specifying proportional mode
  void SetTunings(double, double, double, int);         	  

  // Set the Direction, or "Action" of the controller. DIRECT
  // means the output will increase when error is positive. REVERSE
  // means the opposite.  it's very unlikely that this will be needed
  // once it is set in the constructor.
  void SetControllerDirection(int);

  // Sets the frequency, in Milliseconds, with which 
  // the PID calculation is performed. Default is 100.
  void SetReferenceSampleTime(double);

  // Zero out integral of error
  void clearErrorIntegral();

  double GetKp();
  double GetKi();
  double GetKd();
  bool isOnError();
  bool isEnabled(); // Pause, unpause calculations
  int GetDirection();
  double GetReferenceSampleTime();

private:
  void Initialize();
	
  double dispKp;
  double dispKi;
  double dispKd;
    
  double kp; // (P)roportional Tuning Parameter
  double ki; // (I)ntegral Tuning Parameter
  double kd; // (D)erivative Tuning Parameter

	int controllerDirection;

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
	bool inAuto, pOnE;
};

class PID_FLOAT
{
public:

	static const int8_t DIRECT = 0;
	static const int8_t REVERSE = 1;
	static const int8_t P_ON_M = 0;
	static const int8_t P_ON_E = 1;

  PID_FLOAT(); // Call init

  // Constructor. Links the PID to the Input, Output, and 
  // Setpoint. Initial tuning parameters are also set here.
  // (overload for specifying proportional mode)
  PID_FLOAT(float*, float*, float*, float, float, float, float, int, int);

  // Constructor. Links the PID to the Input, Output, and
  // Setpoint. Initial tuning parameters are also set here.
  PID_FLOAT(float*, float*, float*, float, float, float, float, int);

  void Init(float*, float*, float*, float, float, float, float, int, int);

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
  void SetTunings(float, float, float);

  // Overload for specifying proportional mode
  void SetTunings(float, float, float, int);         	  

  // Set the Direction, or "Action" of the controller. DIRECT
  // means the output will increase when error is positive. REVERSE
  // means the opposite.  it's very unlikely that this will be needed
  // once it is set in the constructor.
  void SetControllerDirection(int);

  // Sets the frequency, in Milliseconds, with which 
  // the PID calculation is performed. Default is 100.
  void SetReferenceSampleTime(float);

  // Zero out integral of error
  void clearErrorIntegral();

  float GetKp();
  float GetKi();
  float GetKd();
  bool isOnError();
  bool isEnabled(); // Pause, unpause calculations
  int GetDirection();
  float GetReferenceSampleTime();

private:
  void Initialize();
	
  float dispKp;
  float dispKi;
  float dispKd;
    
  float kp; // (P)roportional Tuning Parameter
  float ki; // (I)ntegral Tuning Parameter
  float kd; // (D)erivative Tuning Parameter

	int controllerDirection;

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
	bool inAuto, pOnE;
};