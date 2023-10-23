#include <math.h>

#include <Arduino.h> // for constrain(), min(), max()
#include <BBControllers.h>
#include <BBConsole.h>

using namespace bb;

bb::PIDController::PIDController() {
  input_ = NULL;
  output_ = NULL;
}

void bb::PIDController::begin(ControlInput* input, ControlOutput* output) {
  if(begun_) return;

  input_ = input;
  output_ = output;

  setControlParameters(0.0f, 0.0f, 0.0f);
  setIUnbounded();
  setControlUnbounded();
  setGoal(input_->present());

  reset();

  begun_ = true;
}

void bb::PIDController::end() {
  begun_ = false;
}

void bb::PIDController::reset() {
  lastErr_ = errI_ = lastErrD_ = lastControl_ = 0.0f;
}

void bb::PIDController::update(void) {
  if(!begun_) { bb::Console::console.printlnBroadcast("update() without begin()!"); return; }
  float dt = 0.01; // FIXME

  //bb::Runloop::runloop.excuseOverrun();

  float err = goal_ - input_->present();

  errI_ += err * dt;
  if(iBounded_) {
    errI_ = constrain(errI_, iMin_, iMax_);
  }

  lastErrD_ = (lastErr_ - err)/dt;
  lastErr_ = err;

  lastControl_ = kp_ * lastErr_ + ki_ * errI_ + kd_ * lastErrD_;

  if(controlBounded_) {
    lastControl_ = constrain(lastControl_, controlMin_, controlMax_);
  }

  Console::console.printlnBroadcast(String("Control: Goal:") + goal_ + " Cur In:" + input_->present() + " Cur Out:" + output_->present() + " Err:" + err + " ErrI:" + errI_ + " ErrD:" + lastErrD_ + " Control:" + lastControl_);

  output_->set(lastControl_);
}
  
void bb::PIDController::setGoal(const float& sp) {
  goal_ = sp;
}

float bb::PIDController::goal() {
  return goal_;
}

float bb::PIDController::present() {
  if(!begun_) return 0.0f;
  return input_->present();
}

void bb::PIDController::setControlParameters(const float& kp, const float& ki, const float& kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
  
void bb::PIDController::getControlParameters(float& kp, float& ki, float& kd) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
}
  
void bb::PIDController::getControlState(float& err, float& errI, float& errD, float& control) {
  err = lastErr_;
  errI = errI_;
  errD = lastErrD_;
  control = lastControl_;
}

void bb::PIDController::setIBounds(float iMin, float iMax) {
  iBounded_ = true;
  iMin_ = iMin; iMax_ = iMax;
}

void bb::PIDController::setIUnbounded() {
  iMin_ = iMax_ = 0.0f;
  iBounded_ = false;
}
  
bool bb::PIDController::isIBounded() {
  return iBounded_;
}

void bb::PIDController::setControlBounds(float controlMin, float controlMax) {
  controlBounded_ = true;
  controlMin_ = controlMin; controlMax_ = controlMax;
}

void bb::PIDController::setControlUnbounded() {
  controlMin_ = controlMax_ = 0.0f;
  controlBounded_ = false;
}
  
bool bb::PIDController::isControlBounded() {
  return controlBounded_;
}

