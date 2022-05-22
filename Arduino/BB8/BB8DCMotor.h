#if !defined(BB8DCMOTOR_H)
#define BB8DCMOTOR_H

#include <Arduino.h>

class BB8DCMotor {
  public:
  typedef enum {
    DCM_BRAKE,
    DCM_IDLE,
    DCM_FORWARD,
    DCM_BACKWARD
  } Direction;
  
  BB8DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm);

  void setDirectionAndSpeed(Direction dir, uint8_t speed);

  static void setEnablePin(uint8_t pin_en) { 
    BB8DCMotor::pin_en = pin_en;
    pinMode(pin_en, OUTPUT);
    digitalWrite(pin_en, LOW);
    BB8DCMotor::en = false;    
  }
  static void setEnabled(bool en) {
    if(BB8DCMotor::en == en) return;
    if(en) digitalWrite(pin_en, HIGH);
    else digitalWrite(pin_en, LOW);
    BB8DCMotor::en = en;
  }
  static bool isEnabled() { return BB8DCMotor::en; }

  protected:
  uint8_t pin_a, pin_b, pin_pwm;
  Direction dir; 
  uint8_t speed;
  static bool en;
  static uint8_t pin_en;
};

#endif // !defined(BB8MOTORCONTROL_H)
