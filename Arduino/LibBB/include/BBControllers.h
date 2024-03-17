#if !defined(BBCONTROLLERS_H)
#define BBCONTROLLERS_H

#include <sys/types.h>
#include <BBError.h>

namespace bb {

class ControlInput {
public:
  virtual Result update() = 0;
  virtual float present() = 0;
  virtual float controlGain() { return 1.0f; }
};

class ControlOutput {
public:
  virtual float present() = 0;
  virtual Result set(float value) = 0;
};

class PIDController: public ControlOutput {
public:
  PIDController(ControlInput& input, ControlOutput& output);

  void reset(); // reset aggregated errors
  
  virtual void update(void);

  void setGoal(const float& sp);
  void setCurrentAsGoal();
  float goal() { return goal_; }
  float error() { return lastErr_; }
  virtual Result set(float value) { setGoal(value); return RES_OK; }
  virtual float present();

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
  //! Overwrite this if you want to modify the pure control output with something.
  virtual Result setControlOutput(float value) { return output_.set(value); } 

  ControlInput& input_;
  ControlOutput& output_;

  float kp_, ki_, kd_;
  float lastErr_, errI_, lastErrD_, lastControl_;
  float iMin_, iMax_; bool iBounded_;
  float controlMin_, controlMax_; bool controlBounded_;
  float goal_;
  unsigned long lastCycleUS_;
};


};

#endif // BB8CONTROLLERS_H