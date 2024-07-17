#ifndef BRING_SMART_H
#define BRING_SMART_H

#include "Arduino.h"
#include "BringSmartConfig.h"

class BringSmartMotor {
public:
  byte F;
  byte S;

  byte FE, SE, FM, SM;

  bool FlagWheelMode;
  bool FlagJointMode;
  bool FlagInterrupt;
  bool FlagSavePosition;

  unsigned long Timer;

  int HardPosition;
  int HardPositionPrev;
  int HardVelocity;
  int GoalPosition;

  double integralVel;
  double integralVelPrevErr;
  double integralPos;
  double integralPosPrevErr;

  double constrainPWM(double value);
  void calcRealVelocity();
  void controlDriver();
  void velocityPID();
  void positionPID();

  void wheelMode();
  void jointMode();

  int PWM;
  double GoalRadianVelocity;
  double RealRadianVelocity;

  void init(int FE, int SE, int FM, int SM);
  void interruptListener();
  void tick();

  double getRealRadianVelocity();
  double getGoalRadianVelocity();

  void setGoalVelocity(double _GoalRadianVelocity);

};

#endif
