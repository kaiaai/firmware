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

#include "brushed_ctl.h"

void BrushedMotorController::update() {  
  unsigned long tickTime = micros();
  unsigned long tickTimeDelta = tickTime - tickSampleTimePrev;
  if ((tickTimeDelta < pidUpdatePeriodUs) && !setPointHasChanged)
    return;

  tickSampleTimePrev = tickTime;
  
  long int encNow = encoder;
  long int encDelta = encNow - encPrev;
  encPrev = encNow;
  float ticksPerMicroSec = ((float) encDelta) / ((float) tickTimeDelta);
  measuredRPM = ticksPerMicroSec * ticksPerMicroSecToRPM;

  if (targetRPM == 0 && measuredRPM == 0) {
      // Prevent wheels from twitching or slowly turning after stop
      pid.clearErrorIntegral();
  }
  
  float sampleTime = tickTimeDelta * 1e-6;
  pid.Compute(sampleTime);
  setPWM(pidPWM);
}
