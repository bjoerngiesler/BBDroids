#include "DODriveController.h"

DOVelControlOutput::DOVelControlOutput(bb::ControlOutput& left, bb::ControlOutput& right):
  left_(left),
  right_(right) {
    goalVel_ = 0;
    goalRot_ = 0;
    curVel_ = 0;
    curRot_ = 0;
    accel_ = 0;
    maxSpeed_ = 0;
    lastCycleUS_ = micros();
}

bb::Result DOVelControlOutput::set(float value) {
  bb::Result resLeft, resRight;
  unsigned long us = micros();
  float dt;
  if(us < lastCycleUS_) {
    dt = ((ULONG_MAX - lastCycleUS_) + us)/1e6;
  } else {
    dt = (us - lastCycleUS_)/1e6;
  }
  lastCycleUS_ = us;

  if(curVel_ < goalVel_) {
    curVel_ += fabs(accel_*dt);
    if(curVel_ > goalVel_) curVel_ = goalVel_;
  } else {
    curVel_ -= fabs(accel_*dt);
    if(curVel_ < goalVel_) curVel_ = goalVel_;
  }

  if(curRot_ < goalRot_) {
    curRot_ += fabs(accel_*dt);
    if(curRot_ > goalRot_) curRot_ = goalRot_;
  } else {
    curRot_ -= fabs(accel_*dt);
    if(curRot_ < goalRot_) curRot_ = goalRot_;
  }

  float leftGoal = curVel_ + curRot_ - value;
  float rightGoal = curVel_ - curRot_ - value;
  if(!EPSILON(maxSpeed_)) {
    leftGoal = constrain(leftGoal, -maxSpeed_, maxSpeed_);
    rightGoal = constrain(rightGoal, -maxSpeed_, maxSpeed_);
  }

  //Console::console.printfBroadcast("Set goal L%f R%f\n", leftGoal, rightGoal);
  resLeft = left_.set(leftGoal);
  resRight = right_.set(rightGoal);

  if(resLeft != RES_OK) return resLeft;
  if(resRight != RES_OK) return resRight;
  return RES_OK;
}

float DOVelControlOutput::present() {
  return (left_.present() + right_.present()) / 2.0f;
}

void DOVelControlOutput::setGoalVelocity(float goalVel) {
  goalVel_ = goalVel;
  if(EPSILON(accel_)) {
    curVel_ = goalVel_;
  }
}
  
void DOVelControlOutput::setAcceleration(float accel) {
  accel_ = accel;
}
  
void DOVelControlOutput::setGoalRotation(float goalRot) {
  goalRot_ = goalRot;
  if(EPSILON(accel_)) {
    curRot_ = goalRot_;
  }
}

void DOVelControlOutput::setDeadband(float deadband) {
  deadband_ = deadband;  
}

DOVelControlInput::DOVelControlInput(bb::ControlInput& left, bb::ControlInput& right):
  left_(left), right_(right) {  
}

float DOVelControlInput::present() {
  return (left_.present() + right_.present())/2.0f;
}

// ==

DOPosControlInput::DOPosControlInput(bb::Encoder& left, bb::Encoder& right):
  left_(left), right_(right) {
}

float DOPosControlInput::present() {
  return (left_.presentPosition() + right_.presentPosition())/2.0f;
}

// ==

DOPosControlOutput::DOPosControlOutput(DOVelControlOutput& velOut):
  velOut_(velOut) {
}

Result DOPosControlOutput::set(float goal) {
  velOut_.setGoalVelocity(goal);
  velOut_.setGoalRotation(0);
  return RES_OK;
}

float DOPosControlOutput::present() {
  return velOut_.goalVelocity();
}