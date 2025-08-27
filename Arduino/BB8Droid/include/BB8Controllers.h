#if !defined(BB8POSCONTROLOUTPUT_H)
#define BB8POSCONTROLOUTPUT_H

#include <LibBB.h>

class BB8PosCtrlOutput: public bb::ControlOutput {
public:
  BB8PosCtrlOutput(bb::PIDController& balanceController): balanceController_(balanceController) {}
  virtual float present() { return balanceController_.controlOffset(); }
  virtual bb::Result set(float value) { balanceController_.setControlOffset(value); }
protected:
  bb::PIDController& balanceController_;
};

#endif // BB8POSCONTROLOUTPUT_H