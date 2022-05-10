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
  
  DCMotor(uint8_t pin_en, uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm);

  void setEnabled(bool en);
  bool isEnabled() const;
  void setDirectionAndSpeed(Direction dir, uint8_t speed);

  protected:
  uint8_t pin_en, pin_a, pin_b, pin_pwm;
  bool en;
  Direction dir; 
  uint8_t speed;
};

#endif // !defined(MOTORCONTROL_H)
