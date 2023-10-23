#include <BBDCMotor.h>
#include <BBConsole.h>
#include <limits.h>
#include <math.h>

bb::DCMotorControlOutput::DCMotorControlOutput(bb::DCMotor& motor): motor_(motor) {
}

float bb::DCMotorControlOutput::present() {
  switch(motor_.direction()) {
  case bb::DCMotor::DCM_BRAKE:
  case bb::DCMotor::DCM_IDLE:
    return 0;
    break;
  case bb::DCMotor::DCM_BACKWARD:
    return -motor_.speed();
    break;
  case bb::DCMotor::DCM_FORWARD:
  default:
    return motor_.speed();
    break;
  }
}

bool bb::DCMotorControlOutput::set(float value) {
  if(value < 0) {
    motor_.setDirectionAndSpeed(bb::DCMotor::DCM_BACKWARD, -value);
  } else {
    motor_.setDirectionAndSpeed(bb::DCMotor::DCM_FORWARD, value);
  }
  return true;
}

bb::DCMotor::DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pin_pwm_ = pin_pwm;
  pin_en_ = pin_en;
  scheme_ = SCHEME_A_B_PWM;

  if(pin_en_ != PIN_OFF)
    en_ = false;
  else 
    en_ = true;
  dir_ = DCM_IDLE;
  speed_ = 0;
}

bb::DCMotor::DCMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b) {
  pin_a_ = pin_pwm_a;
  pin_b_ = pin_pwm_b;
  pin_pwm_ = PIN_OFF;
  pin_en_ = PIN_OFF;
  scheme_ = SCHEME_PWM_A_PWM_B;

  en_ = true;
  dir_ = DCM_IDLE;
  speed_ = 0;
}

bool bb::DCMotor::begin() {
  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);

  if(pin_pwm_ != PIN_OFF)
    pinMode(pin_pwm_, OUTPUT);
  if(pin_en_ != PIN_OFF)
    pinMode(pin_en_, OUTPUT);

  if(scheme_ == SCHEME_A_B_PWM) {
    digitalWrite(pin_a_, LOW);
    digitalWrite(pin_b_, LOW);
    analogWrite(pin_pwm_, 0);
    if(pin_en_ != PIN_OFF) {
      digitalWrite(pin_en_, LOW);
      en_ = false;
    } else {
      en_ = true;
    }
  } else {
    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, 0);
  }

  return true;
}

void bb::DCMotor::setEnabled(bool en) {
  if(pin_en_ == PIN_OFF) return;

  if (en_ == en) return;
  if (en) digitalWrite(pin_en_, HIGH);
  else digitalWrite(pin_en_, LOW);
  en_ = en;
}

void bb::DCMotor::setDirectionAndSpeed(DCMotor::Direction dir, uint8_t speed) {
  if(scheme_ == SCHEME_A_B_PWM) {
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
  } else {
    switch(dir) {
    case DCM_BRAKE:
    case DCM_IDLE:
      analogWrite(pin_a_, 0);
      analogWrite(pin_b_, 0);
      speed = 0;
      break;
    case DCM_FORWARD:
      analogWrite(pin_a_, 0);
      analogWrite(pin_b_, speed);
      break;
    case DCM_BACKWARD:
      analogWrite(pin_a_, speed);
      analogWrite(pin_b_, 0);
      break;
    default:
      break;
    }
  }

  dir_ = dir;
  speed_ = speed;
}

#if defined(ARDUINO_ARCH_SAMD)

bb::EncoderMotor::EncoderMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en, uint8_t pin_enc_a, uint8_t pin_enc_b)
  : bb::DCMotor(pin_a, pin_b, pin_pwm, pin_en),
    enc_(pin_enc_a, pin_enc_b) {
  mode_ = CONTROL_PWM;
  mmPT_ = 1.0;
  maxSpeed_ = 1.0;
  presentPWM_ = 0;
  presentSpeed_ = 0;
  presentPos_ = 0;
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

bb::EncoderMotor::EncoderMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b, uint8_t pin_enc_a, uint8_t pin_enc_b)
  : bb::DCMotor(pin_pwm_a, pin_pwm_b),
    enc_(pin_enc_a, pin_enc_b) {
  mode_ = CONTROL_PWM;
  mmPT_ = 1.0;
  maxSpeed_ = 1.0;
  presentPWM_ = 0;
  presentSpeed_ = 0;
  presentPos_ = 0;
  accel_ = 1; // FIXME this is neither in ticks/s^2 nor in mm/s^2 but in pwm/cycle. Nasty nasty.
  
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
  presentPos_ += lastCycleTicks_;

  unsigned long us = micros();
  unsigned long timediffUS;
  if (us < lastCycleUS_) {
    timediffUS = (ULONG_MAX - lastCycleUS_) + us;
  } else {
    timediffUS = us - lastCycleUS_;
  }
  lastCycleUS_ = us;

  presentSpeed_ = ((double)lastCycleTicks_ / (double)timediffUS)*1e6;

  //bb::Console::console.printlnBroadcast(String("Last cycle ticks: ") + lastCycleTicks_ + " Time difference: " + timediffUS + " Present speed: " + presentSpeed_);

  switch(mode_) {
  case CONTROL_SPEED:
    speedControlUpdate((float)timediffUS / 1e6);
    break;
  case CONTROL_POSITION:
    positionControlUpdate((float)timediffUS / 1e6);
    break;
  case CONTROL_PWM:
  default:
    pwmControlUpdate((float)timediffUS / 1e6);
    break;
  }
}

void bb::EncoderMotor::pwmControlUpdate(float dt) {
  if (goal_ < presentPWM_) {
    presentPWM_ -= accel_;
    if (presentPWM_ < goal_) presentPWM_ = goal_;
  } else if (goal_ > presentPWM_) {
    presentPWM_ += accel_;
    if (presentPWM_ > goal_) presentPWM_ = goal_;
  }

  presentPWM_ = constrain(presentPWM_, -255.0, 255.0);

  if (presentPWM_ < 0) setDirectionAndSpeed(DCM_BACKWARD, -presentPWM_);
  else setDirectionAndSpeed(DCM_FORWARD, presentPWM_);
}
  
void bb::EncoderMotor::speedControlUpdate(float dt) {
  #define EPSILON(x,y) (fabs(x-y)<0.01)
  static unsigned int numZero = 0;
  float err;

  err = goal_ - presentSpeed_;

  if(EPSILON(err, 0) && EPSILON(goal_, 0) && EPSILON(presentSpeed_, 0)) {
    numZero++;
  } else {
    numZero = 0;
  }

  if(numZero > 10) { // 1s
    errSpeedI_ = 0;
    presentPWM_ = 0;
    setDirectionAndSpeed(DCM_IDLE, 0);
    return;
  }

  errSpeedI_ += err * dt;

  errSpeedD_ = (err - errSpeedL_)/dt; // differentiate

  controlSpeed_ = kpSpeed_ * err + kiSpeed_ * errSpeedI_ + kdSpeed_ * errSpeedD_;
  presentPWM_ = controlSpeed_;
  
#if 1
  bb::Console::console.printlnBroadcast(String("present speed: ") + presentSpeed_ + 
    " Goal: " + goal_ + " Err: " + err + " ErrI: " + errSpeedI_ + 
    " ErrD: " + errSpeedD_ + " Control: " + controlSpeed_ + " PWM: " + presentPWM_ + " dt: " + dt);
#endif

  float bias = 0; // 40;
  if(EPSILON(presentPWM_, 0)) {
    setDirectionAndSpeed(DCM_IDLE, 0);
  } else if(presentPWM_ < 0) {
    float speed = constrain(-presentPWM_+bias, 0, 255.0);
    setDirectionAndSpeed(DCM_BACKWARD, speed);
  } else if(presentPWM_ > 0) {
    float speed = constrain(presentPWM_+bias, 0, 255.0);
    setDirectionAndSpeed(DCM_FORWARD, speed);
  } 

  errSpeedL_ = err;
}

void bb::EncoderMotor::positionControlUpdate(float dt) {
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

  if(mode_ != mode) {
    errSpeedI_ = errSpeedL_ = 0;
    controlSpeed_ = 0;
    errPosI_ = errPosL_ = 0;
    mode_ = mode;
  }
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

float bb::EncoderMotor::getPresentSpeed(bb::EncoderMotor::Unit unit) { 
  if(unit == UNIT_TICKS) return presentSpeed_; 
  return presentSpeed_ * mmPT_;
}
  
float bb::EncoderMotor::getPresentPosition(bb::EncoderMotor::Unit unit) { 
  if(unit == UNIT_TICKS) return presentPos_; 
  return presentPos_ * mmPT_;
}

bb::DriveControlState bb::EncoderMotor::getDriveControlState() {
  bb::DriveControlState state;
  
  state.errorState = ERROR_OK;
  state.presentPWM = presentPWM_;
  state.presentSpeed = presentSpeed_;
  state.presentPos = presentPos_;
  state.controlMode = mode_;
  state.goal = goal_;
  switch(mode_) {
  case CONTROL_SPEED:
    state.err = errSpeedL_;
    state.errI = errSpeedI_;
    state.errD = errSpeedD_;
    state.control = controlSpeed_;
    break;
  case CONTROL_POSITION:
    state.err = errPosL_;
    state.errI = errPosI_;
    state.errD = errPosD_;
    state.control = controlPos_;    
    break;
  default:
    state.err = state.errI = state.errD = state.control = 0;
  }

  return state;
}


#endif // ARDUINO_ARCH_SAMD
