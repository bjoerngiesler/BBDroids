#include "BB8DCMotor.h"
#include "BB8Config.h"

BB8DCMotor::BB8DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en) {
  this->pin_a = pin_a;
  this->pin_b = pin_b;
  this->pin_pwm = pin_pwm;
  this->pin_en = pin_en;

  pinMode(pin_a, OUTPUT);
  digitalWrite(pin_a, LOW);
  pinMode(pin_b, OUTPUT);
  digitalWrite(pin_b, LOW);
  pinMode(pin_pwm, OUTPUT);
  analogWrite(pin_pwm, 0);
  pinMode(pin_en, OUTPUT);
  digitalWrite(pin_en, LOW);

  en = false;
  dir = DCM_IDLE;
  speed = 0;
}

void BB8DCMotor::setEnabled(bool en) {
  if(this->en == en) return;
  if(en) digitalWrite(pin_en, HIGH);
  else digitalWrite(pin_en, LOW);
  this->en = en;
}

void BB8DCMotor::setDirectionAndSpeed(BB8DCMotor::Direction dir, uint8_t speed) {
  switch(dir) {
    case DCM_BRAKE:
      digitalWrite(pin_a, HIGH);
      digitalWrite(pin_b, HIGH);
      speed = 0;
      analogWrite(pin_pwm, 0);
      break;

    case DCM_IDLE:
      digitalWrite(pin_a, LOW);
      digitalWrite(pin_b, LOW);
      speed = 0;
      analogWrite(pin_pwm, 0);
      break;

    case DCM_FORWARD:
      digitalWrite(pin_a, HIGH);
      digitalWrite(pin_b, LOW);
      analogWrite(pin_pwm, speed);
      break;

    case DCM_BACKWARD:
      digitalWrite(pin_a, LOW);
      digitalWrite(pin_b, HIGH);
      analogWrite(pin_pwm, speed);
      break;

    default:
      break;
  }

  this->dir = dir;
  this->speed = speed;
}
