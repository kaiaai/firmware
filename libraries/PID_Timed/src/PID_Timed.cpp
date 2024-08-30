// Arduino PID Timed Library
// Based on https://github.com/br3ttb/Arduino-PID-Library
// This Library is licensed under a GPLv3 License

#include <PID_Timed.h>

PID::PID() {}

PID::PID(double* Input, double* Output, double* Setpoint,
  double Kp, double Ki, double Kd, double referenceSampleTime,
  int POn, int ControllerDirection)
{
  Init(Input, Output, Setpoint, Kp, Ki, Kd, referenceSampleTime, POn, ControllerDirection);
}

PID::PID(double* Input, double* Output, double* Setpoint,
  double Kp, double Ki, double Kd, double referenceSampleTime, int ControllerDirection)
  :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, referenceSampleTime, P_ON_E, ControllerDirection)
{
}

void PID::Init(double* Input, double* Output, double* Setpoint,
  double Kp, double Ki, double Kd, double referenceSampleTime,
  int POn, int ControllerDirection) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  // Default output limit corresponds to the arduino pwm limits
  PID::SetOutputLimits(0, 255);

  ReferenceSampleTime = referenceSampleTime;

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd, POn);
}

// This, as they say, is where the magic happens.  this function should be called
// every time "void loop()" executes.  the function will decide for itself whether a new
// pid Output needs to be computed.  returns true when the output is computed,
// false when nothing has been done.
bool PID::Compute(double SampleTime)
{
  if(!inAuto) return false;
   
  double ratio  = SampleTime / ReferenceSampleTime;
  ki = dispKi * ratio;
  kd = dispKd / ratio;
   
  // Compute all the working error variables
  double input = *myInput;
  double error = *mySetpoint - input;
  double dInput = (input - lastInput);
  outputSum+= (ki * error);

  // Add Proportional on Measurement, if P_ON_M is specified
  if(!pOnE) outputSum-= kp * dInput;

  if(outputSum > outMax) outputSum= outMax;
  else if(outputSum < outMin) outputSum= outMin;

  // Add Proportional on Error, if P_ON_E is specified
  double output;
  if(pOnE) output = kp * error;
  else output = 0;

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

void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
  if (Kp<0 || Ki<0 || Kd<0) return;

  pOnE = POn == P_ON_E;

  kp = Kp;
  ki = Ki;
  kd = Kd;

  if (controllerDirection == REVERSE)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
   }
   dispKp = kp; dispKi = ki; dispKd = kd;
}

void PID::SetTunings(double Kp, double Ki, double Kd) {
  SetTunings(Kp, Ki, Kd, pOnE ? P_ON_E : P_ON_M); 
}

void PID::SetReferenceSampleTime(double SampleTime)
{
  ReferenceSampleTime = SampleTime;
}

double PID::GetReferenceSampleTime()
{
  return ReferenceSampleTime;
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
void PID::SetControllerDirection(int Direction)
{
  if(inAuto && Direction !=controllerDirection)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  controllerDirection = Direction;
}

double PID::GetKp() { return  dispKp; }
double PID::GetKi() { return  dispKi; }
double PID::GetKd() { return  dispKd; }
bool PID::isOnError() { return pOnE; }
bool PID::isEnabled() { return inAuto; }
int PID::GetDirection() { return controllerDirection; }

PID_FLOAT::PID_FLOAT() {}

PID_FLOAT::PID_FLOAT(float* Input, float* Output, float* Setpoint,
  float Kp, float Ki, float Kd, float referenceSampleTime,
  int POn, int ControllerDirection)
{
  Init(Input, Output, Setpoint, Kp, Ki, Kd, referenceSampleTime, POn, ControllerDirection);
}

PID_FLOAT::PID_FLOAT(float* Input, float* Output, float* Setpoint,
  float Kp, float Ki, float Kd, float referenceSampleTime, int ControllerDirection)
  :PID_FLOAT::PID_FLOAT(Input, Output, Setpoint, Kp, Ki, Kd, referenceSampleTime, P_ON_E, ControllerDirection)
{
}

void PID_FLOAT::Init(float* Input, float* Output, float* Setpoint,
  float Kp, float Ki, float Kd, float referenceSampleTime,
  int POn, int ControllerDirection) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  // Default output limit corresponds to the arduino pwm limits
  PID_FLOAT::SetOutputLimits(0, 255);

  ReferenceSampleTime = referenceSampleTime;

  PID_FLOAT::SetControllerDirection(ControllerDirection);
  PID_FLOAT::SetTunings(Kp, Ki, Kd, POn);
}

// This, as they say, is where the magic happens.  this function should be called
// every time "void loop()" executes.  the function will decide for itself whether a new
// pid Output needs to be computed.  returns true when the output is computed,
// false when nothing has been done.
bool PID_FLOAT::Compute(float SampleTime)
{
  if(!inAuto) return false;
   
  float ratio  = SampleTime / ReferenceSampleTime;
  ki = dispKi * ratio;
  kd = dispKd / ratio;
   
  // Compute all the working error variables
  float input = *myInput;
  float error = *mySetpoint - input;
  float dInput = (input - lastInput);
  outputSum+= (ki * error);

  // Add Proportional on Measurement, if P_ON_M is specified
  if(!pOnE) outputSum-= kp * dInput;

  if(outputSum > outMax) outputSum= outMax;
  else if(outputSum < outMin) outputSum= outMin;

  // Add Proportional on Error, if P_ON_E is specified
  float output;
  if(pOnE) output = kp * error;
  else output = 0;

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

void PID_FLOAT::SetTunings(float Kp, float Ki, float Kd, int POn)
{
  if (Kp<0 || Ki<0 || Kd<0) return;

  pOnE = POn == P_ON_E;

  kp = Kp;
  ki = Ki;
  kd = Kd;

  if (controllerDirection == REVERSE)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
   }
   dispKp = kp; dispKi = ki; dispKd = kd;
}

void PID_FLOAT::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, pOnE ? P_ON_E : P_ON_M); 
}

void PID_FLOAT::SetReferenceSampleTime(float SampleTime)
{
  ReferenceSampleTime = SampleTime;
}

float PID_FLOAT::GetReferenceSampleTime()
{
  return ReferenceSampleTime;
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
void PID_FLOAT::SetControllerDirection(int Direction)
{
  if(inAuto && Direction !=controllerDirection)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  controllerDirection = Direction;
}

float PID_FLOAT::GetKp() { return  dispKp; }
float PID_FLOAT::GetKi() { return  dispKi; }
float PID_FLOAT::GetKd() { return  dispKd; }
bool PID_FLOAT::isOnError() { return pOnE; }
bool PID_FLOAT::isEnabled() { return inAuto; }
int PID_FLOAT::GetDirection() { return controllerDirection; }
