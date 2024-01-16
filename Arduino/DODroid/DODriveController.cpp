#include "DODriveController.h"

DODriveControlOutput::DODriveControlOutput(bb::ControlOutput& left, bb::ControlOutput& right):
  left_(left),
  right_(right) {
    goalVel_ = 0;
    goalRot_ = 0;
    accel_ = 0;
    presentGoalVelL_ = 0;
    presentGoalVelR_ = 0;
    lastCycleUS_ = micros();
}

bb::Result DODriveControlOutput::set(float value) {
  bb::Result resLeft, resRight;
  static int zeroControlTimes = 0;

  float l = goalVel_ + goalRot_ - value;
  float r = goalVel_ - goalRot_ - value;

  resLeft = left_.set(l);
  resRight = right_.set(r);

  if(resLeft != RES_OK) return resLeft;
  if(resRight != RES_OK) return resRight;
  return RES_OK;
}
  
float DODriveControlOutput::present() {
  return (left_.present() + right_.present()) / 2.0f;
}

void DODriveControlOutput::setGoalVelocity(float goalVel) {
  goalVel_ = goalVel;
}
  
void DODriveControlOutput::setAcceleration(float accel) {
  accel_ = accel;
}
  
void DODriveControlOutput::setGoalRotation(float goalRot) {
  goalRot_ = goalRot;
}

void DODriveControlOutput::setDeadband(float deadband) {
  deadband_ = deadband;  
}