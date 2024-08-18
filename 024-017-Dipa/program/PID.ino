#include "PID.h"

PIDv3::PIDv3() {
  pOn = P_ON_E;
  setMode(AUTOMATIC);
  inAuto = false;
  SampleTime = 100;
  lastTime = millis() - SampleTime;
}

bool PIDv3::compute(float mySetpoint, float myInput) {
  if (!inAuto) return false;
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= SampleTime) {
    /*Compute all the working error variables*/
    float input = myInput;
    float error = mySetpoint - input;
    float dInput = (input - lastInput);
    outputSum += (ki * error);
    /*Add Proportional on Measurement, if P_ON_M is specified*/
    if (!pOnE) outputSum -= kp * dInput;
    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
    /*Add Proportional on Error, if P_ON_E is specified*/
    float output;
    if (pOnE) output = kp * error;
    else output = 0;
    /*Compute Rest of PIDv3 Output*/
    output += outputSum - kd * dInput;
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    myOutput = output;
    /*Remember some variables for next time*/
    lastInput = input;
    lastTime = now;
    return true;
  } else return false;
}

void PIDv3::setTunings(float Kp, float Ki, float Kd, int POn) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  pOn = POn;
  pOnE = POn == P_ON_E;
  dispKp = Kp;
  dispKi = Ki;
  dispKd = Kd;
  float SampleTimeInSec = ((float)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
  if (controllerDirection == REVERSE) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

void PIDv3::setSampleTime(float NewSampleTime) {
  if (NewSampleTime > 0) {
    float ratio = (float)NewSampleTime / (float)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = NewSampleTime;
  }
}

void PIDv3::setOutputLimits(float Min, float Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;
  if (inAuto) {
    if (myOutput > outMax) myOutput = outMax;
    else if (myOutput < outMin) myOutput = outMin;
    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
  }
}

void PIDv3::setMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto) {
    PIDv3::initialize();
  }
  inAuto = newAuto;
}

void PIDv3::initialize() {
  outputSum = myOutput;
  lastInput = 0;  //
  if (outputSum > outMax) outputSum = outMax;
  else if (outputSum < outMin) outputSum = outMin;
}

void PIDv3::setControllerDirection(int Direction) {
  if (inAuto && Direction != controllerDirection) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  controllerDirection = Direction;
}

float PIDv3::getKp() {
  return dispKp;
}

float PIDv3::getKi() {
  return dispKi;
}

float PIDv3::getKd() {
  return dispKd;
}

float PIDv3::getOutput() {
  return myOutput;
}

int PIDv3::getMode() {
  return inAuto ? AUTOMATIC : MANUAL;
}

int PIDv3::getDirection() {
  return controllerDirection;
}