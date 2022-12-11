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
  
  BB8DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en);
  void setDirectionAndSpeed(Direction dir, uint8_t speed);  
  void setEnabled(bool en);
  bool isEnabled() { return en; }

  protected:
  uint8_t pin_a, pin_b, pin_pwm, pin_en;
  Direction dir; 
  uint8_t speed;
  bool en;
};

#endif // !defined(BB8MOTORCONTROL_H)
