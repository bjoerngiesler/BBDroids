#if !defined(BB8POSCONTROLOUTPUT_H)
#define BB8POSCONTROLOUTPUT_H

#include <LibBB.h>

class BB8PosCtrlOutput: public bb::ControlOutput {
public:
  BB8PosCtrlOutput(bb::PIDController& balanceController): balanceController_(balanceController) {}
  virtual float present() { return balanceController_.controlOffset(); }
  virtual bb::Result set(float value) { balanceController_.setControlOffset(value); return bb::RES_OK; }
protected:
  bb::PIDController& balanceController_;
};

// Body roll controller. Commands the roll servo (position mode) as
//
//   tilt  = kff*goal + head + kp*err + ki*integral(err) - kRate*gyroRollRate     [body/pendulum degrees]
//   servo = center + reduction*tilt                                       [servo degrees]
//
// - Feedforward (kff) gives the open-loop lean; P/I only correct the remainder.
// - Damping acts on the measured gyro rate rather than the derivative of the
//   error, so joystick steps do not cause derivative kicks, and the rate is not
//   delayed by the attitude filter. The base class kd is not used (forced to 0).
// - Gains are in body degrees; the gear reduction is applied once at the output.
class BB8RollController: public bb::PIDController {
public:
  BB8RollController(bb::ControlInput& input, bb::ControlOutput& output, bb::IMU& imu);

  void setRollParameters(float kp, float ki, float kRate, float kff);
  void setOutputMapping(float reduction, float center) { reduction_ = reduction; center_ = center; }
  void setRateFilterCutoff(float cutoff) { rateFilter_.setCutoff(cutoff); }
  // Additional feedforward tilt (body degrees), e.g. to counteract dome motion. Set every cycle before update().
  void setHeadCompensation(float tilt) { headTilt_ = tilt; }
  // Use this instead of setDebug(): the base class debug output calls
  // input.present() a second time, which steps IMUControlInput's filter twice.
  void setRollDebug(bool yesno) { rollDebug_ = yesno; }

  virtual void update() override;
  float rate() { return rate_; }

protected:
  virtual bb::Result setControlOutput(float value) override;

  bb::IMU& imu_;
  bb::LowPassFilter rateFilter_;
  float kRate_, kff_, reduction_, center_, rate_, headTilt_;
  bool rollDebug_;
};

#endif // BB8POSCONTROLOUTPUT_H