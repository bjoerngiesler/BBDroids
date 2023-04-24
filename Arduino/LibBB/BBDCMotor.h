#if !defined(BBDCMOTOR_H)
#define BBDCMOTOR_H

#include <Arduino.h>
#include <Encoder.h>

namespace bb {

class DCMotor {
public:
  typedef enum {
    DCM_BRAKE,
    DCM_IDLE,
    DCM_FORWARD,
    DCM_BACKWARD
  } Direction;

  DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en);

  virtual bool begin();

  virtual void setDirectionAndSpeed(Direction dir, uint8_t speed);
  virtual void setEnabled(bool en);
  virtual bool isEnabled() {
    return en_;
  }

protected:
  uint8_t pin_a_, pin_b_, pin_pwm_, pin_en_;
  Direction dir_;
  uint8_t speed_;
  bool en_;
};

class EncoderMotor: public DCMotor {
public:
  typedef enum {
    CONTROL_PWM,
    CONTROL_SPEED,
    CONTROL_POSITION
  } ControlMode;

  typedef enum {
    UNIT_MILLIMETERS,
    UNIT_TICKS
  } Unit;

  EncoderMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en, uint8_t pin_enc_a, uint8_t pin_enc_b);

  void setReverse(bool reverse);

  void setMillimetersPerTick(float mmPT);
  void setMaxSpeed(float maxSpeed, Unit unit = UNIT_MILLIMETERS);
  void setAcceleration(float accel, Unit unit = UNIT_MILLIMETERS);

  // If mode is CONTROL_PWM, unit is disregarded.
  void setGoal(float goal, ControlMode mode, Unit unit = UNIT_MILLIMETERS); 
  float getGoal(Unit unit = UNIT_MILLIMETERS);
  ControlMode getControlMode();

  void setSpeedControlParameters(float kp, float ki, float kd);
  void getSpeedControlParameters(float &kp, float &ki, float &kd) { kp = kpSpeed_; ki = kiSpeed_; kd = kdSpeed_; }
  void getSpeedControlState(float& err, float& errI, float& errD, float& control) { err = errSpeedL_; errI = errSpeedI_; errD = errSpeedD_; control = controlSpeed_; }
  void setPosControlParameters(float kp, float ki, float kd);
  void getPosControlParameters(float &kp, float &ki, float &kd) { kp = kpPos_; ki = kiPos_; kd = kdPos_; }
  float getCurrentPWM() { return currentPWM_; }
  float getCurrentSpeed(Unit unit = UNIT_MILLIMETERS);
  float getCurrentPosition(Unit unit = UNIT_MILLIMETERS);

  void update();

  long getLastCycleTicks() { return lastCycleTicks_; }

protected:
  void pwmControlUpdate();
  void speedControlUpdate();
  void positionControlUpdate();

  Encoder enc_;
  float mmPT_;
  long lastCycleTicks_;
  unsigned long lastCycleUS_;
  float maxSpeed_;

  ControlMode mode_;

  float accel_;
  float goal_;
  float currentPWM_;
  float currentSpeed_; // internally always in encoder ticks per second
  long currentPos_;    // internally always in encoder ticks

  float kpSpeed_, kiSpeed_, kdSpeed_;
  float errSpeedI_, errSpeedL_;
  float errSpeedD_, controlSpeed_;
  float kpPos_, kiPos_, kdPos_;
  float errPosI_, errPosL_;

  bool reverse_;
};

}; // namespace bb

#endif  // !defined(BBDCMOTOR_H)
