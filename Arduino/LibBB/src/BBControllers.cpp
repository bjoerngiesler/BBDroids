#include <math.h>

#include <Arduino.h> // for constrain(), min(), max()
#include <LibBB.h>

using namespace bb;

bb::PIDController::PIDController(ControlInput& input, ControlOutput& output): 
  input_(input), 
  output_(output), 
  autoupdate_(true),
  debug_(false),
  inputScale_(1.0),
  reverse_(false),
  inhibit_(false),
  ramp_(0.0),
  differentialFilter_(5.0, 100) {
  setControlParameters(0.0f, 0.0f, 0.0f);
  setIUnbounded();
  setControlUnbounded();
  setGoal(input_.present());
  deadbandMin_ = deadbandMax_ = 0;
  errDeadbandMin_ = errDeadbandMax_ = 0;
  controlOffset_ = 0;

  reset();
}

void bb::PIDController::reset() {
  lastErr_ = errI_ = lastErrD_ = lastControl_ = 0.0f;
  lastCycleUS_ = micros();
}

void bb::PIDController::update(void) {
  unsigned long us = micros();
  unsigned long timediffUS;
  if (us < lastCycleUS_) {
    timediffUS = (ULONG_MAX - lastCycleUS_) + us;
  } else {
    timediffUS = us - lastCycleUS_;
  }
  lastCycleUS_ = us;

  if(autoupdate_) input_.update();

  float dt = timediffUS / 1e6; 

  if(!EPSILON(ramp_) && !EPSILON(curSetpoint_ - goal_)) {
    if(curSetpoint_ < goal_) {
      curSetpoint_ += fabs(ramp_*dt);
      if(curSetpoint_ > goal_) {
        curSetpoint_ = goal_;
      }
    } else if(curSetpoint_ > goal_) {
      curSetpoint_ -= fabs(ramp_*dt);
      if(curSetpoint_ < goal_) {
        curSetpoint_ = goal_;
      }
    }
  }

  //bb::Runloop::runloop.excuseOverrun();

  float err;
  if(reverse_) {
    err = curSetpoint_ + input_.present()*inputScale_;
  } else {
    err = curSetpoint_ - input_.present()*inputScale_;
  } 
  if(err > errDeadbandMin_ && err < errDeadbandMax_) err = 0;

  errI_ += err * dt;
  if(iBounded_) {
    errI_ = constrain(errI_, iMin_, iMax_);
  }

  lastErrD_ = (err - lastErr_)/dt;
  lastErr_ = err;
  lastErrDFiltered_ = differentialFilter_.filter(lastErrD_);

  lastControl_ = kp_ * lastErr_ + ki_ * errI_ + kd_ * lastErrDFiltered_;
  lastControl_ *= input_.controlGain();

  if(controlBounded_) {
    lastControl_ = constrain(lastControl_, controlMin_, controlMax_);
  }

  float controlOut;
  if(lastControl_ > deadbandMin_ && lastControl_ < deadbandMax_) {
    controlOut = controlOffset_; 
  } else if(reverse_) {
    controlOut = -(lastControl_ + controlOffset_);
  } else {
    controlOut = lastControl_ + controlOffset_;
  }
  if(debug_)
    Console::console.printfBroadcast("Control: SP: %f Cur In: %f Cur Out: %f Err: %f errI: %f errD: %f Control: %f\n", curSetpoint_, input_.present(), output_.present(), err, errI_, lastErrDFiltered_, lastControl_);
  if(inhibit_ == false) setControlOutput(controlOut);
}
  
void bb::PIDController::setGoal(const float& sp) {
  goal_ = sp;
  if(EPSILON(ramp_)) curSetpoint_ = goal_;
}

void bb::PIDController::setPresentAsGoal() {
  goal_ = present();
  if(EPSILON(ramp_)) curSetpoint_ = goal_;
}

float bb::PIDController::present() {
  return input_.present();
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
  errD = lastErrDFiltered_;
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

void bb::PIDController::setControlDeadband (float deadbandMin, float deadbandMax) {
  deadbandMin_ = deadbandMin; deadbandMax_ = deadbandMax;
}

void bb::PIDController::setErrorDeadband(float errDeadbandMin, float errDeadbandMax) {
  errDeadbandMin_ = errDeadbandMin; 
  errDeadbandMax_ = errDeadbandMax;
}

