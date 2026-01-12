#if !defined(BBDCMOTOR_H)
#define BBDCMOTOR_H

#include <Arduino.h>
#include <BBError.h>
#include <BBPacket.h>
#include <BBControllers.h>
#include <BBLowPassFilter.h>

namespace bb {

class LowPassFilter; // forward declaration

/*!
  \brief DC Motor class.

  This class represents a DC motor controlled using 8-bit PWM. There are two basic schemes it supports - one that uses two
  pins to control the motor direction and one pin for PWM speed (with an additional optional enable pin), and one that uses
  just two PWM pins for direction and speed simultaneously. 

  The class inherits from \ref ControlOutput, so can be used as output in a control scheme. It can be set to be reversed.
  There is also a deadband parameter that causes the output PWM to be pulled to zero if it is very low, which can be used to 
  reduce motor noise at standstill.
*/
class DCMotor: public ControlOutput {
public:
  static const uint8_t PIN_OFF = 255;

  enum Scheme {
    SCHEME_A_B_PWM,
    SCHEME_PWM_A_PWM_B
  };

  // Use to initialize a motor that has A/B/PWM control scheme
  DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en = PIN_OFF);
  // Use to initialize a motor that has PWM_A/PWM_B control scheme
  DCMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b);

  // speed will be constrained to -255.0..255.0. 
  // Negative numbers run the motor backward. 
  // Between (-1..1) the motor will idle.
  virtual Result set(float speed);
  virtual Result brake(float force=10.0);
  virtual float present() { return speed_; }
  virtual void setEnabled(bool en);
  virtual bool isEnabled() { return en_; }
  virtual void setReverse(bool reverse_);
  virtual bool isReverse() { return reverse_; }

  virtual void setEndstop(uint8_t endstop, uint8_t mode);
  virtual bool brakeIfEndstop(float force=10.0);

  virtual uint8_t deadband() { return deadband_; }
  virtual void setDeadband(uint8_t db) { deadband_ = db; }

  // Use this to provide a custom analogWrite() function to the motor, e.g. if you're doing your own timer setup
  void setCustomAnalogWrite(void(*cust)(uint8_t pin, uint8_t dutycycle));
  void write(uint8_t pin, uint8_t dutycycle);

protected:

  void (*customAnalogWrite_)(uint8_t pin, uint8_t dutycycle);

  uint8_t pin_a_, pin_b_, pin_pwm_, pin_en_;
  uint8_t pin_endstop_, endstop_mode_;
  uint8_t deadband_;
  float speed_;
  bool en_, reverse_;
  Scheme scheme_;
};

}; // namespace bb

#endif  // !defined(BBDCMOTOR_H)
