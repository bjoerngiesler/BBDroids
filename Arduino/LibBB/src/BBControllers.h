#if !defined(BBCONTROLLERS_H)
#define BBCONTROLLERS_H

#include <sys/types.h>
#include <BBError.h>
#include <BBLowPassFilter.h>

namespace bb {

class ControlInput {
public:
  virtual float present() = 0;
  virtual Result update() = 0;
  virtual float controlGain() { return 1.0f; }
};

class ControlOutput {
public:
  virtual float present()         = 0;
  virtual Result set(float value) = 0;
};

class PIDController: public ControlOutput {
public:
  PIDController(ControlInput& input, ControlOutput& output);

  void reset(); // reset aggregated errors
  
  virtual void update(void);

  //! If this is set to true, the PID controller will call the input's update() method during its own update().
  //! Use for simple setups, but not if you need input in several locations.
  //! True by default.
  void setAutoUpdate(bool yesno) { autoupdate_ = yesno; }
  bool doesAutoUpdate() { return autoupdate_; }

  void setDebug(bool yesno) { debug_ = yesno; }

  void setGoal(const float& sp);
  void setPresentAsGoal();
  float goal() { return goal_; }
  float error() { return lastErr_; }
  virtual Result set(float value) { setGoal(value); return RES_OK; }
  virtual float present();

  void setControlParameters(const float& kp, const float& ki, const float& kd);
  void getControlParameters(float& kp, float& ki, float& kd);
  void getControlState(float& err, float& errI, float& errD, float& control);
  void setControlOffset(float offset) { controlOffset_ = offset; }
  float controlOffset() { return controlOffset_; }

  void setIBounds(float iMin, float iMax);
  void setIUnbounded();
  bool isIBounded();

  void setControlBounds(float controlMin, float controlMax);
  void setControlUnbounded();
  bool isControlBounded();

  void setControlDeadband(float deadbandMin, float deadbandMax);
  void setErrorDeadband(float errDeadbandMin, float errDeadbandMax);

  void setInputScaleFactor(float inputScale) { inputScale_ = inputScale; }
  float inputScaleFactor() { return inputScale_; }

  void setReverse(bool yesno) { reverse_ = yesno; }
  bool reverse() { return reverse_; }

  void setInhibit(bool yesno) { inhibit_ = yesno; }
  bool inhibit() { return inhibit_; }

  void setRamp(float ramp) { ramp_ = ramp; }
  float ramp(void) { return ramp_; }

protected:
  //! Overwrite this if you want to modify the pure control output with something.
  virtual Result setControlOutput(float value) { return output_.set(value); } 

  ControlInput& input_;
  ControlOutput& output_;
  bool autoupdate_, debug_;
  float inputScale_;
  bool reverse_;
  bool inhibit_;

  float kp_, ki_, kd_;
  float lastErr_, errI_, lastErrD_, lastErrDFiltered_, lastControl_;
  float iMin_, iMax_; bool iBounded_;
  float controlMin_, controlMax_; bool controlBounded_;
  float deadbandMin_, deadbandMax_;
  float errDeadbandMin_, errDeadbandMax_;
  float goal_, ramp_, curSetpoint_;
  unsigned long lastCycleUS_;
  float controlOffset_;
  bb::LowPassFilter differentialFilter_;
};


};

#endif // BB8CONTROLLERS_H