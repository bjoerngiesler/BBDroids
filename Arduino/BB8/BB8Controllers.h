#if !defined(BB8CONTROLLERS_H)
#define BB8CONTROLLERS_H

#include <sys/types.h>
#include "BB8ConfigStorage.h"
#include "BB8Config.h"

class BB8ControlInput {
public:
  virtual float getValue() = 0;
};

class BB8BodyIMUControlInput: public BB8ControlInput {
public:
  typedef enum {
    IMU_PROBE_ROLL,
    IMU_PROBE_PITCH,
    IMU_PROBE_HEADING
  } ProbeType;

  BB8BodyIMUControlInput(ProbeType pt);
  float getValue();
protected:
  ProbeType pt_;
};

class BB8ControlOutput {
public:
  virtual bool available() = 0;
  virtual bool setValue(float value) = 0;
  virtual bool enable(bool onoff) = 0;
  virtual bool isEnabled() = 0;
};

class BB8ServoControlOutput: public BB8ControlOutput {
public:
  BB8ServoControlOutput(uint8_t servoNum, float offset);
  bool available();
  bool setValue(float value);
  bool enable(bool onoff);
  bool isEnabled();

protected:
  uint8_t sn_;
  float offset_;
};

class BB8PIDController {
public:
  static BB8PIDController rollController;

  bool begin(BB8ConfigStorage::Controller c);
  bool available();
  void setSetpoint(float sp);
  bool step(bool outputForPlot = false);
  bool enable(bool onoff);
  bool isEnabled();

protected:
  BB8PIDController();
  bool takeValuesFromStorage(BB8ConfigStorage::Controller c);
  BB8ControlInput* input_; 
  BB8ControlOutput* output_;
  float kp_, ki_, kd_;
  float imax_, iabort_, deadband_;
  float setpoint_;
  bool enabled_;

  float last_e_, i_;
  BB8ConfigStorage::Controller controller_storage_;
};

#endif // BB8CONTROLLERS_H