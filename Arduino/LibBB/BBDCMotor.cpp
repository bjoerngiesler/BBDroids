#include <BBDCMotor.h>
#include <BBConsole.h>
#include <limits.h>
#include <math.h>

static unsigned int MIN_PWM_TO_MOVE = 0; // 20;

bb::DCMotor::DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pin_pwm_ = pin_pwm;
  pin_en_ = pin_en;

  en_ = false;
  dir_ = DCM_IDLE;
  speed_ = 0;
}

bool bb::DCMotor::begin() {
  pinMode(pin_a_, OUTPUT);
  digitalWrite(pin_a_, LOW);
  pinMode(pin_b_, OUTPUT);
  digitalWrite(pin_b_, LOW);
  pinMode(pin_pwm_, OUTPUT);
  analogWrite(pin_pwm_, 0);
  pinMode(pin_en_, OUTPUT);
  digitalWrite(pin_en_, LOW);

  return true;
}

void bb::DCMotor::setEnabled(bool en) {
  if (en_ == en) return;
  if (en) digitalWrite(pin_en_, HIGH);
  else digitalWrite(pin_en_, LOW);
  en_ = en;
}

void bb::DCMotor::setDirectionAndSpeed(DCMotor::Direction dir, uint8_t speed) {
  switch (dir) {
    case DCM_BRAKE:
      digitalWrite(pin_a_, HIGH);
      digitalWrite(pin_b_, HIGH);
      speed = 0;
      analogWrite(pin_pwm_, 0);
      break;

    case DCM_IDLE:
      digitalWrite(pin_a_, LOW);
      digitalWrite(pin_b_, LOW);
      speed = 0;
      analogWrite(pin_pwm_, 0);
      break;

    case DCM_FORWARD:
      digitalWrite(pin_a_, HIGH);
      digitalWrite(pin_b_, LOW);
      analogWrite(pin_pwm_, speed);
      break;

    case DCM_BACKWARD:
      digitalWrite(pin_a_, LOW);
      digitalWrite(pin_b_, HIGH);
      analogWrite(pin_pwm_, speed);
      break;

    default:
      break;
  }

  dir_ = dir;
  speed_ = speed;
}

bb::EncoderMotor::EncoderMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en, uint8_t pin_enc_a, uint8_t pin_enc_b)
  : bb::DCMotor(pin_a, pin_b, pin_pwm, pin_en),
    enc_(pin_enc_a, pin_enc_b) {
  mode_ = CONTROL_PWM;
  mmPT_ = 1.0;
  maxSpeed_ = 1.0;
  currentPWM_ = 0;
  currentSpeed_ = 0;
  currentPos_ = 0;
  accel_ = 1;
  
  kpSpeed_ = 0.001;
  kiSpeed_ = 0.0004;
  kdSpeed_ = 0.0;
  errSpeedI_ = 0.0;

  kpPos_ = 1.0;
  kiPos_ = 0.0;
  kdPos_ = 0.0;
  errPosI_ = 0.0;

  reverse_ = false;
}

void bb::EncoderMotor::setReverse(bool reverse) {
  reverse_ = reverse;
}

void bb::EncoderMotor::setMillimetersPerTick(float mmPT) { 
  mmPT_ = mmPT; 
}

void bb::EncoderMotor::setMaxSpeed(float maxSpeed, bb::EncoderMotor::Unit unit) { 
  if(unit == UNIT_TICKS) maxSpeed_ = maxSpeed; 
  else maxSpeed_ = maxSpeed / mmPT_;
}

void bb::EncoderMotor::setAcceleration(float accel, bb::EncoderMotor::Unit unit) { 
  if(unit == UNIT_TICKS) accel_ = accel; 
  else accel_ = accel / mmPT_;
}


void bb::EncoderMotor::update() {
  lastCycleTicks_ = enc_.read();
  enc_.write(0);
  if(reverse_) lastCycleTicks_ = -lastCycleTicks_;
  currentPos_ += lastCycleTicks_;

  unsigned long us = micros();
  unsigned long timediffUS;
  if (us < lastCycleUS_) {
    timediffUS = (ULONG_MAX - lastCycleUS_) + us;
  } else {
    timediffUS = us - lastCycleUS_;
  }
  lastCycleUS_ = us;

  currentSpeed_ = ((double)lastCycleTicks_ / (double)timediffUS)*1e6;

  switch(mode_) {
  case CONTROL_SPEED:
    speedControlUpdate();
    break;
  case CONTROL_POSITION:
    positionControlUpdate();
    break;
  case CONTROL_PWM:
  default:
    pwmControlUpdate();
    break;
  }
}

void bb::EncoderMotor::pwmControlUpdate() {
  //bb::Console::console.printlnBroadcast(String("Last cycle ticks: ") + lastCycleTicks_ + " Time difference: " + timediffUS + " Current speed: " + currentSpeed_);
  if (goal_ < currentPWM_) {
    currentPWM_ -= accel_;
    if (currentPWM_ < goal_) currentPWM_ = goal_;
  } else if (goal_ > currentPWM_) {
    currentPWM_ += accel_;
    if (currentPWM_ > goal_) currentPWM_ = goal_;
  }

  currentPWM_ = constrain(currentPWM_, -255.0, 255.0);

  if (currentPWM_ < 0) setDirectionAndSpeed(DCM_BACKWARD, -currentPWM_);
  else setDirectionAndSpeed(DCM_FORWARD, currentPWM_);
}
  
void bb::EncoderMotor::speedControlUpdate() {
  float err;

  err = goal_ - currentSpeed_;

  errSpeedI_ = constrain(errSpeedI_ + err, -255.0, 255.0);

  errSpeedD_ = errSpeedL_ - err; // differentiate

  controlSpeed_ = kpSpeed_ * err + kiSpeed_ * errSpeedI_ + kdSpeed_ * errSpeedD_;
  currentPWM_ += controlSpeed_;

#if 0
  bb::Console::console.printlnBroadcast(String("Current speed: ") + currentSpeed_ + 
    " Goal: " + goal_ + " Err: " + err + " ErrI: " + errSpeedI_ + 
    " ErrD: " + errSpeedD_ + " Control: " + controlSpeed_ + " PWM: " + currentPWM_);
#endif

  if(currentPWM_ < 0) {
    float speed = constrain(-currentPWM_ + MIN_PWM_TO_MOVE, 0, 255.0);
    setDirectionAndSpeed(DCM_BACKWARD, speed);
  } else {
    float speed = constrain(currentPWM_ + MIN_PWM_TO_MOVE, 0, 255.0);
    setDirectionAndSpeed(DCM_FORWARD, speed);
  } 

  errSpeedL_ = err;
}

void bb::EncoderMotor::positionControlUpdate() {
  bb::Console::console.printlnBroadcast("Position control is not yet implemented!");
}

void bb::EncoderMotor::setGoal(float goal, bb::EncoderMotor::ControlMode mode, bb::EncoderMotor::Unit unit) {
  switch(mode) {
  case CONTROL_SPEED:
    if(unit == UNIT_TICKS) goal_ = goal;
    else goal_ = goal / mmPT_;
    break;

  case CONTROL_POSITION:
    if(unit == UNIT_TICKS) goal_ = goal;
    else goal_ = goal / mmPT_;
    break;

  case CONTROL_PWM:
  default:
    goal_ = goal;
    break;
  }

  errSpeedI_ = errSpeedL_ = 0;
  errPosI_ = errPosL_ = 0;
  mode_ = mode;
}

float bb::EncoderMotor::getGoal(bb::EncoderMotor::Unit unit) {
  if(unit == UNIT_TICKS) return goal_;
  return goal_ * mmPT_;
}

bb::EncoderMotor::ControlMode bb::EncoderMotor::getControlMode() { 
  return mode_;
}

void bb::EncoderMotor::setSpeedControlParameters(float kp, float ki, float kd) {
  kpSpeed_ = kp; kiSpeed_ = ki; kdSpeed_ = kd;
}

void bb::EncoderMotor::setPosControlParameters(float kp, float ki, float kd) {
  kpSpeed_ = kp; kiSpeed_ = ki; kdSpeed_ = kd;
}

float bb::EncoderMotor::getCurrentSpeed(bb::EncoderMotor::Unit unit) { 
  if(unit == UNIT_TICKS) return currentSpeed_; 
  return currentSpeed_ * mmPT_;
}
  
float bb::EncoderMotor::getCurrentPosition(bb::EncoderMotor::Unit unit) { 
  if(unit == UNIT_TICKS) return currentPos_; 
  return currentPos_ * mmPT_;
}

