#include <BBDCMotor.h>
#include <BBConsole.h>

bb::DCMotor::DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pin_pwm_ = pin_pwm;
  pin_en_ = pin_en;
  scheme_ = SCHEME_A_B_PWM;

  pin_endstop_ = PIN_OFF;
  endstop_mode_ = HIGH;

  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);

  if(pin_pwm_ != PIN_OFF)
    pinMode(pin_pwm_, OUTPUT);
  if(pin_en_ != PIN_OFF)
    pinMode(pin_en_, OUTPUT);

  digitalWrite(pin_a_, LOW);
  digitalWrite(pin_b_, LOW);
  if(pin_en_ != PIN_OFF) {
    digitalWrite(pin_en_, LOW);
    en_ = false;
  } else {
    en_ = true;
  }

  speed_ = 0;
  reverse_ = false;

  customAnalogWrite_ = nullptr;
}

bb::DCMotor::DCMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b) {
  pin_a_ = pin_pwm_a;
  pin_b_ = pin_pwm_b;
  pin_pwm_ = PIN_OFF;
  pin_en_ = PIN_OFF;
  scheme_ = SCHEME_PWM_A_PWM_B;

  en_ = true;
  
  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);

  digitalWrite(pin_a_, LOW);
  digitalWrite(pin_b_, LOW);

  if(pin_pwm_ != PIN_OFF)
    pinMode(pin_pwm_, OUTPUT);
  if(pin_en_ != PIN_OFF)
    pinMode(pin_en_, OUTPUT);

  speed_ = 0;

  customAnalogWrite_ = nullptr;
}

void bb::DCMotor::setEnabled(bool en) {
  if(pin_en_ == PIN_OFF) return;

  if (en_ == en) return;
  if (en) digitalWrite(pin_en_, HIGH);
  else digitalWrite(pin_en_, LOW);
  en_ = en;
}

bb::Result bb::DCMotor::set(float speed) {
  if(fabs(speed) < deadband_) speed = 0;
  speed_ = constrain(speed, -255.0, 255.0);
  if(reverse_) speed_ = -speed_;

  if(scheme_ == SCHEME_A_B_PWM) {
    if(speed_ > -1 && speed_ < 1) { // idle
      digitalWrite(pin_a_, LOW);
      digitalWrite(pin_b_, LOW);
      speed = 0;
      write(pin_pwm_, 0);
    } else if(speed_ > 0) { // forward
      digitalWrite(pin_a_, HIGH);
      digitalWrite(pin_b_, LOW);
      write(pin_pwm_, speed_);
    } else { // backward
      digitalWrite(pin_a_, LOW);
      digitalWrite(pin_b_, HIGH);
      write(pin_pwm_, -speed_);
    }
  } else {
    if(speed_ > -1 && speed_ <1) {
      write(pin_a_, 0);
      write(pin_b_, 0);
      speed_ = 0;
    } else if(speed_ > 0) { // forward
      write(pin_a_, 0);
      write(pin_b_, speed_);
    } else { // backward
      write(pin_a_, -speed_);
      write(pin_b_, 0);
    }
  }

  return RES_OK;
}

bb::Result bb::DCMotor::brake(float force) {
  speed_ = 0;

  if(scheme_ == SCHEME_A_B_PWM) {
    digitalWrite(pin_a_, HIGH);
    digitalWrite(pin_b_, HIGH);
    write(pin_pwm_, constrain(force, 0, 255));
  } else {
    write(pin_a_, constrain(force, 0, 255));
    write(pin_b_, constrain(force, 0, 255));
  }

  return RES_OK;
}

void bb::DCMotor::setReverse(bool reverse) {
  reverse_ = reverse;
}

void bb::DCMotor::setCustomAnalogWrite(void(*cust)(uint8_t pin, uint8_t dutycycle)) {
  customAnalogWrite_ = cust;
}

void bb::DCMotor::write(uint8_t pin, uint8_t dutycycle) {
  if(customAnalogWrite_ != nullptr) customAnalogWrite_(pin, dutycycle);
  else analogWrite(pin, dutycycle);
}

void bb::DCMotor::setEndstop(uint8_t endstop, uint8_t mode) {
  pin_endstop_ = endstop;
  endstop_mode_ = mode;
}

bool bb::DCMotor::brakeIfEndstop(float force) {
  if(digitalRead(pin_endstop_) == endstop_mode_) {
    brake(force);
    return true;
  }
  return false;
}
