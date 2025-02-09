// Arduino PID Timed Library
// Based on https://github.com/br3ttb/Arduino-PID-Library
// This Library is licensed under a GPLv3 License

#include <PID_Timed.h>

PID::PID() {}

PID::PID(double* Input, double* Output, double* Setpoint,
  double Kpe, double Ki, double Kd, double referenceSampleTime,
  double Kpm=0)
{
  Init(Input, Output, Setpoint, Kpe, Ki, Kd, referenceSampleTime, Kpm);
}

void PID::Init(double* Input, double* Output, double* Setpoint,
  double Kpe, double Ki, double Kd, double referenceSampleTime,
  double Kpm=0) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  // Default output limit corresponds to the arduino pwm limits
  PID::SetOutputLimits(0, 255);

  ReferenceSampleTime = referenceSampleTime;

  PID::SetTunings(Kpe, Ki, Kd, Kpm);
}

// This, as they say, is where the magic happens.  this function should be called
// every time "void loop()" executes.  the function will decide for itself whether a new
// pid Output needs to be computed.  returns true when the output is computed,
// false when nothing has been done.
bool PID::Compute(double SampleTime)
{
  if(!inAuto) return false;

  // Approximate process as linear when SampleTime ~= ReferenceSampleTime
  double ratio  = SampleTime / ReferenceSampleTime;

  // Compute all the working error variables
  double input = *myInput;
  double error = *mySetpoint - input; // absolute error
  double dInput = input - lastInput; // -dError
  dInput /= ratio;
  outputSum += ki * error * ratio;

  // Add Proportional on Measurement
  outputSum -= kpm * dInput;

  if(outputSum > outMax) outputSum= outMax;
  else if(outputSum < outMin) outputSum= outMin;

  // Add Proportional on Error
  double output = kpe * error;

  // Compute Rest of PID Output
  output += outputSum - kd * dInput;

  if(output > outMax) output = outMax;
  else if(output < outMin) output = outMin;
  *myOutput = output;

  // Remember some variables for next time
  lastInput = input;
  return true;
}

void PID::clearErrorIntegral() {
  outputSum = 0;
  lastInput = *myInput;
}

void PID::SetTunings(double Kpe, double Ki, double Kd, double Kpm=0)
{
  if (Kpe<0 || Ki<0 || Kd<0 || Kpm<0) return;

  kpe = Kpe;
  kpm = Kpm;
  ki = Ki;
  kd = Kd;
}

void PID::SetReferenceSampleTime(double SampleTime)
{
  ReferenceSampleTime = SampleTime;
}

void PID::SetOutputLimits(double Min, double Max)
{
  if(Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if(inAuto)
  {
	 if (*myOutput > outMax)
      *myOutput = outMax;
	 else if (*myOutput < outMin)
      *myOutput = outMin;

	 if (outputSum > outMax)
      outputSum= outMax;
	 else if (outputSum < outMin)
      outputSum= outMin;
   }
}

void PID::enable(bool en)
{
  if (en && !inAuto)
  {
    PID::Initialize(); // we just went from manual to auto
  }
  inAuto = en;
}

//	Does all the things that need to happen to ensure a bumpless transfer
// from manual to automatic mode.
void PID::Initialize()
{
  outputSum = *myOutput;
  lastInput = *myInput;
  if(outputSum > outMax) outputSum = outMax;
  else if(outputSum < outMin) outputSum = outMin;
}

// The PID will either be connected to a DIRECT acting process (+Output leads
// to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
// know which one, because otherwise we may increase the output when we should
// be decreasing.  This is called from the constructor.

// For REVERSE-acting process, set ki, kd, kpe, kpm to -ki, -kd, -kpe, -kpm

double PID::GetKpe() { return  kpe; }
double PID::GetKpm() { return  kpm; }
double PID::GetKi() { return  ki; }
double PID::GetKd() { return  kd; }
bool PID::isEnabled() { return inAuto; }
double PID::GetReferenceSampleTime() { return ReferenceSampleTime; }

PID_FLOAT::PID_FLOAT() {}

PID_FLOAT::PID_FLOAT(float* Input, float* Output, float* Setpoint,
  float Kpe, float Ki, float Kd, float referenceSampleTime, float Kpm=0)
{
  Init(Input, Output, Setpoint, Kpe, Ki, Kd, referenceSampleTime, Kpm);
}

void PID_FLOAT::Init(float* Input, float* Output, float* Setpoint,
  float Kpe, float Ki, float Kd, float referenceSampleTime, float Kpm=0) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  // Default output limit corresponds to the arduino pwm limits
  PID_FLOAT::SetOutputLimits(0, 255);

  ReferenceSampleTime = referenceSampleTime;

  PID_FLOAT::SetTunings(Kpe, Ki, Kd, Kpm);
}

// This, as they say, is where the magic happens.  this function should be called
// every time "void loop()" executes.  the function will decide for itself whether a new
// pid Output needs to be computed.  returns true when the output is computed,
// false when nothing has been done.
bool PID_FLOAT::Compute(float SampleTime)
{
  if(!inAuto) return false;

  // Approximate process is linear when SampleTime ~= ReferenceSampleTime
  float ratio  = SampleTime / ReferenceSampleTime;
   
  // Compute all the working error variables
  float input = *myInput;
  float error = *mySetpoint - input; // absolute error
  float dInput = input - lastInput; // -dError
  dInput /= ratio;
  outputSum += ki * ratio * error;

  // Add Proportional on Measurement
  outputSum -= kpm * dInput;

  if(outputSum > outMax) outputSum= outMax;
  else if(outputSum < outMin) outputSum= outMin;

  // Add Proportional on Error
  float output = kpe * error;

  // Compute Rest of PID Output
  output += outputSum - kd * dInput;

  if(output > outMax) output = outMax;
  else if(output < outMin) output = outMin;
  *myOutput = output;

  // Remember some variables for next time
  lastInput = input;
  return true;
}

void PID_FLOAT::clearErrorIntegral() {
  outputSum = 0;
  lastInput = *myInput;
}

void PID_FLOAT::SetTunings(float Kpe, float Ki, float Kd, float Kpm=0)
{
  if (Kpe<0 || Ki<0 || Kd<0 || Kpm<0) return;

  kpe = Kpe;
  kpm = Kpm;
  ki = Ki;
  kd = Kd;
}

void PID_FLOAT::SetReferenceSampleTime(float SampleTime)
{
  ReferenceSampleTime = SampleTime;
}

void PID_FLOAT::SetOutputLimits(float Min, float Max)
{
  if(Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if(inAuto)
  {
	 if (*myOutput > outMax)
      *myOutput = outMax;
	 else if (*myOutput < outMin)
      *myOutput = outMin;

	 if (outputSum > outMax)
      outputSum= outMax;
	 else if (outputSum < outMin)
      outputSum= outMin;
   }
}

void PID_FLOAT::enable(bool en)
{
  if (en && !inAuto)
  {
    PID_FLOAT::Initialize(); // we just went from manual to auto
  }
  inAuto = en;
}

//	Does all the things that need to happen to ensure a bumpless transfer
// from manual to automatic mode.
void PID_FLOAT::Initialize()
{
  outputSum = *myOutput;
  lastInput = *myInput;
  if(outputSum > outMax) outputSum = outMax;
  else if(outputSum < outMin) outputSum = outMin;
}

// The PID will either be connected to a DIRECT acting process (+Output leads
// to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
// know which one, because otherwise we may increase the output when we should
// be decreasing.  This is called from the constructor.

// For REVERSE-acting process, set ki, kd, kpe, kpm to -ki, -kd, -kpe, -kpm

float PID_FLOAT::GetKpe() { return  kpe; }
float PID_FLOAT::GetKpm() { return  kpm; }
float PID_FLOAT::GetKi() { return  ki; }
float PID_FLOAT::GetKd() { return  kd; }
bool PID_FLOAT::isEnabled() { return inAuto; }
float PID_FLOAT::GetReferenceSampleTime() { return ReferenceSampleTime; }
