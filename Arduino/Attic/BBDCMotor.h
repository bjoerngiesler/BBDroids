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

  typedef enum {
    SCHEME_A_B_PWM,
    SCHEME_PWM_A_PWM_B
  } Scheme;

  // Use to initialize a motor that has A/B/PWM control scheme
  DCMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en = PIN_OFF);
  // Use to initialize a motor that has PWM_A/PWM_B control scheme
  DCMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b);

  // speed will be constrained to -255.0..255.0. 
  // Negative numbers run the motor backward. 
  // Between (-1..1) the motor will idle.
  virtual Result set(float speed);
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

#if defined(ARDUINO_ARCH_SAMD)

class EncoderMotor: public DCMotor {
public:
  enum ControlMode {
    CONTROL_PWM,
    CONTROL_SPEED,
    CONTROL_POSITION
  };

  enum Unit {
    UNIT_MILLIMETERS,
    UNIT_TICKS
  };

  //EncoderMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en, uint8_t pin_enc_a, uint8_t pin_enc_b);
  //EncoderMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b, uint8_t pin_enc_a, uint8_t pin_enc_b);
  EncoderMotor(uint8_t pin_a, uint8_t pin_b, uint8_t pin_pwm, uint8_t pin_en, Encoder& enc);
  EncoderMotor(uint8_t pin_pwm_a, uint8_t pin_pwm_b, Encoder& enc);

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
  float getPresentPWM() { return presentPWM_; }
  float getPresentSpeed(Unit unit = UNIT_MILLIMETERS);
  float getPresentPosition(Unit unit = UNIT_MILLIMETERS);

  bb::DriveControlState getDriveControlState();

  void update();

  long getLastCycleTicks() { return lastCycleTicks_; }

protected:
  void pwmControlUpdate(float dt);
  void speedControlUpdate(float dt);
  void positionControlUpdate(float dt);

  Encoder& enc_;
  float mmPT_;
  long lastCycleTicks_;
  unsigned long lastCycleUS_;
  float maxSpeed_;

  ControlMode mode_;

  float accel_;

  float goal_;
  float presentPWM_;
  float presentSpeed_; // internally always in encoder ticks per second
  long presentPos_;    // internally always in encoder ticks

  float kpSpeed_, kiSpeed_, kdSpeed_;
  float errSpeedL_, errSpeedI_, errSpeedD_, controlSpeed_;
  float kpPos_, kiPos_, kdPos_;
  float errPosL_, errPosI_, errPosD_, controlPos_;

  bool reverse_;
};


class EncoderControlInput: public ControlInput {
public:
  enum InputMode {
    INPUT_SPEED    = 0,
    INPUT_POSITION = 1
  };

  enum Unit {
    UNIT_MILLIMETERS = 0,
    UNIT_TICKS       = 1
  };

  EncoderControlInput(uint8_t pin_enc_a, uint8_t pin_enc_b, InputMode mode = INPUT_POSITION, Unit unit = UNIT_TICKS);
  
  void setMode(InputMode mode);
  void setUnit(Unit unit);
  InputMode mode() { return mode_; }
  Unit unit() { return unit_; }
  virtual float present();
  virtual float present(InputMode mode, Unit unit, bool raw = false);
  virtual float presentPosition(Unit unit = UNIT_TICKS, bool raw = false);
  virtual float presentSpeed(Unit unit = UNIT_TICKS, bool raw = false);
  virtual Result update();

  float speedFilterCutoff();
  void setSpeedFilterCutoff(float co);

  float positionFilterCutoff();
  void setPositionFilterCutoff(float co);

  void setMillimetersPerTick(float mmPT) { mmPT_ = mmPT; }

  float controlGain() { if(unit_ == UNIT_TICKS) return 1.0f; else return 1/mmPT_; }

protected:
  InputMode mode_;
  Unit unit_;
  Encoder enc_; // FIXME -- since this requires SAMD, possibly replace by own encoder handling?
  bb::LowPassFilter filtSpeed_, filtPos_;

  float mmPT_;
  long lastCycleTicks_;
  unsigned long lastCycleUS_;
  long presentPos_;    // internally always in encoder ticks
  long presentPosFiltered_;
  float presentSpeed_; // internally always in encoder ticks per second
  float presentSpeedFiltered_;
};


#endif // ARDUINO_ARCH_SAMD


}; // namespace bb

#endif  // !defined(BBDCMOTOR_H)
