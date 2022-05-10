#if !defined(DCMOTOR_H)
#define DCMOTOR_H

#include <Arduino.h>

class DCMotor {
  public:
  typedef enum {
    DCM_BRAKE,
    DCM_IDLE,
    DCM_FORWARD,
    DCM_BACKWARD
  } Direction;
  
  DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm);

  void setDirectionAndSpeed(Direction dir, uint8_t speed);

  static void setEnablePin(uint8_t pin_en) { 
    DCMotor::pin_en = pin_en;
    pinMode(pin_en, OUTPUT);
    digitalWrite(pin_en, LOW);
    DCMotor::en = false;    
  }
  static void setEnabled(bool en) {
    if(DCMotor::en == en) return;
    if(en) digitalWrite(pin_en, HIGH);
    else digitalWrite(pin_en, LOW);
    DCMotor::en = en;
  }
  static bool isEnabled() { return DCMotor::en; }

  protected:
  uint8_t pin_a, pin_b, pin_pwm;
  Direction dir; 
  uint8_t speed;
  static bool en;
  static uint8_t pin_en;
};

#endif // !defined(MOTORCONTROL_H)
