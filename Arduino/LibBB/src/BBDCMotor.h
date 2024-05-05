#if !defined(BBDCMOTOR_H)
#define BBDCMOTOR_H

#include <Arduino.h>
#include <BBError.h>
#if defined(ARDUINO_ARCH_SAMD)
#include <Encoder.h>
#endif
#include <BBPacket.h>
#include <BBControllers.h>
#include <BBLowPassFilter.h>

namespace bb {

class LowPassFilter; // forward declaration

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

protected:
  uint8_t pin_a_, pin_b_, pin_pwm_, pin_en_;
  float speed_;
  bool en_, reverse_;
  Scheme scheme_;
};

}; // namespace bb

#endif  // !defined(BBDCMOTOR_H)
