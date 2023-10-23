#if !defined(BBCONTROLLERS_H)
#define BBCONTROLLERS_H

#include <sys/types.h>

namespace bb {

class ControlInput {
public:
  virtual float present() { return 0.0f; }
};

class ControlOutput {
public:
  virtual float present() { return 0.0f; }
  virtual bool set(float value) { (void)value; return false; }
};

class PIDController {
public:
  PIDController();

  void begin(ControlInput* input, ControlOutput* output);
  void end();

  void reset(); // reset aggregated errors
  
  void update(void);

  void setGoal(const float& sp);
  void setCurrentAsGoal();
  float goal();
  float present();

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
  ControlInput* input_;
  ControlOutput* output_;

  bool begun_;

  float kp_, ki_, kd_;
  float lastErr_, errI_, lastErrD_, lastControl_;
  float iMin_, iMax_; bool iBounded_;
  float controlMin_, controlMax_; bool controlBounded_;
  float goal_;
};


};

#endif // BB8CONTROLLERS_H