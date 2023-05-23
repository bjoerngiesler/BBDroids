#if !defined(BB8CONTROLLERS_H)
#define BB8CONTROLLERS_H

#include <sys/types.h>
#include "BB8Config.h"

class BB8ControlInput {
public:
  virtual float current() { return 0.0f; }
};

class BB8ControlOutput {
public:
  virtual float current() { return 0.0f; }
  virtual bool set(float value) { (void)value; return false; }
};

class BB8PIDController {
public:
  BB8PIDController();

  void begin(BB8ControlInput* input, BB8ControlOutput* output);
  void end();
  
  void update(void);

  void setGoal(const float& sp);
  void setCurrentAsGoal();
  float goal();
  float current();

  void setControlParameters(const float& kp, const float& ki, const float& kd);
  void getControlParameters(float& kp, float& ki, float& kd);
  void getControlState(float& err, float& errI, float& errD, float& control);

  void setIBounds(float iMin, float iMax);
  void setIUnbounded();
  bool isIBounded();

  void setControlBounds(float controlMin, float controlMax);
  void setControlUnbounded();
  bool isControlBounded();

protected:
  BB8ControlInput* input_;
  BB8ControlOutput* output_;

  bool begun_;

  float kp_, ki_, kd_;
  float lastErr_, errI_, lastErrD_, lastControl_;
  float iMin_, iMax_; bool iBounded_;
  float controlMin_, controlMax_; bool controlBounded_;
  float goal_;
};

class BB8IMUControlInput: public BB8ControlInput {
public:
  typedef enum {
    IMU_ROLL,
    IMU_PITCH,
    IMU_HEADING
  } ProbeType;

  BB8IMUControlInput(ProbeType pt);
  float current();
protected:
  ProbeType pt_;
};

class BB8ServoControlOutput: public BB8ControlOutput {
public:
  BB8ServoControlOutput(uint8_t servoNum, float offset=0.0f);
  float current();
  bool set(float value);

protected:
  uint8_t sn_;
  float offset_;
};

#endif // BB8CONTROLLERS_H