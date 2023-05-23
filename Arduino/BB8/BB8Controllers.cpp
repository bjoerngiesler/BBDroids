#include <math.h>

#include "BB8Controllers.h"
#include "BB8IMU.h"
#include "BB8Servos.h"

BB8PIDController::BB8PIDController() {
  input_ = NULL;
  output_ = NULL;
}

void BB8PIDController::begin(BB8ControlInput* input, BB8ControlOutput* output) {
  if(begun_) return;

  input_ = input;
  output_ = output;

  setGoal(input_->current());
  setControlParameters(0.0f, 0.0f, 0.0f);
  setIUnbounded();
  setControlUnbounded();
  lastErr_ = errI_ = lastErrD_ = lastControl_ = 0.0f;

  begun_ = true;
}

void BB8PIDController::end() {
  begun_ = false;
}

void BB8PIDController::update(void) {
  if(!begun_) { Console::console.printlnBroadcast("update() without begin()!"); return; }

  float err = goal_ - input_->current();

  errI_ += err;
  if(iBounded_) {
    errI_ = constrain(errI_, iMin_, iMax_);
  }

  lastErrD_ = lastErr_ - err;
  lastErr_ = err;

  lastControl_ = kp_ * lastErr_ + ki_ * errI_ + kd_ * lastErrD_;

  if(controlBounded_) {
    lastControl_ = constrain(lastControl_, controlMin_, controlMax_);
  }

  Console::console.printlnBroadcast(String("Control: Err:") + err + " ErrI:" + errI_ + " ErrD:" + lastErrD_ + " Control:" + lastControl_);

  output_->set(lastControl_);
}
  
void BB8PIDController::setGoal(const float& sp) {
  goal_ = sp;
}

float BB8PIDController::goal() {
  return goal_;
}

float BB8PIDController::current() {
  if(!begun_) return 0.0f;
  return input_->current();
}

void BB8PIDController::setControlParameters(const float& kp, const float& ki, const float& kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
  
void BB8PIDController::getControlParameters(float& kp, float& ki, float& kd) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
}
  
void BB8PIDController::getControlState(float& err, float& errI, float& errD, float& control) {
  err = lastErr_;
  errI = errI_;
  errD = lastErrD_;
  control = lastControl_;
}

void BB8PIDController::setIBounds(float iMin, float iMax) {
  iBounded_ = true;
  iMin_ = iMin; iMax_ = iMax;
}

void BB8PIDController::setIUnbounded() {
  iMin_ = iMax_ = 0.0f;
  iBounded_ = false;
}
  
bool BB8PIDController::isIBounded() {
  return iBounded_;
}

void BB8PIDController::setControlBounds(float controlMin, float controlMax) {
  controlBounded_ = true;
  controlMin_ = controlMin; controlMax_ = controlMax;
}

void BB8PIDController::setControlUnbounded() {
  controlMin_ = controlMax_ = 0.0f;
  controlBounded_ = false;
}
  
bool BB8PIDController::isControlBounded() {
  return controlBounded_;
}

BB8IMUControlInput::BB8IMUControlInput(BB8IMUControlInput::ProbeType pt) {
  pt_ = pt;
}

float BB8IMUControlInput::current() {
  float r, p, h;
  if(BB8IMU::imu.getFilteredRPH(r, p, h) == false) return 0.0f;
  
  switch(pt_) {
  case IMU_ROLL:
    return r;
    break;
  case IMU_PITCH:
    return p;
    break;
  case IMU_HEADING:
    return h;
    break;
  }

  return 0.0f;
}

BB8ServoControlOutput::BB8ServoControlOutput(uint8_t sn, float offset) {
  sn_ = sn;
  offset_ = offset;
}

bool BB8ServoControlOutput::set(float value) {
  return BB8Servos::servos.setGoal(sn_, BB8Servos::servos.present(sn_) - value);
}

float BB8ServoControlOutput::current() {
  return BB8Servos::servos.present(sn_);
}