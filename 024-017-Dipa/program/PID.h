#pragma once

#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

class PIDv3 {
public:
  PIDv3();
  void setMode(int Mode);
  bool compute(float mySetpoint, float myInput);
  void setOutputLimits(float Min, float Max);
  void setTunings(float Kp, float Ki, float Kd, int POn = P_ON_E);
  void setControllerDirection(int Direction);
  void setSampleTime(float NewSampleTime);
  float getKp();
  float getKi();
  float getKd();
  float getOutput();
  int getMode();
  int getDirection();

private:
  void initialize();
  float dispKp;
  float dispKi;
  float dispKd;
  float kp;
  float ki;
  float kd;
  float myOutput;
  int controllerDirection;
  int pOn;
  unsigned long lastTime;
  float outputSum, lastInput;
  float SampleTime;
  float outMin, outMax;
  bool inAuto, pOnE;
};