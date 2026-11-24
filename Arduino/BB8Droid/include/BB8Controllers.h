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
  // I bleed on the way back towards zero (see BB8Params::rollIBleedTau). tau <= 0 disables it.
  void setReturnParameters(float iBleedTau, float window) { iBleedTau_ = iBleedTau; returnWindow_ = window; }
  // Input shaper on the roll goal: 0 = off, 1 = ZV, 2 = ZVD; period in seconds.
  void setShaper(int type, float period) { shaperType_ = type; shaperT_ = period; }
  // Use this instead of setGoal() for the remote roll goal; the (optionally shaped) value is
  // passed on to setGoal() in update().
  void setRollGoal(float goal) { rawGoal_ = goal; }
  float rawGoal() { return rawGoal_; }
  void setOutputMapping(float reduction, float center) { reduction_ = reduction; center_ = center; }
  void setRateFilterCutoff(float cutoff) { rateFilter_.setCutoff(cutoff); }
  // Speed scheduling of the gyro damping: above kdSpeed (mm/s), kRate is scaled by kdSpeed/|speed|.
  // When cornering at speed, lean -> yaw -> centrifugal force makes the roll plane much stiffer and faster;
  // full damping then asks for more tilt acceleration than the servo profile delivers, and the lag turns
  // the damping into excitation (sim/README.md, section 3.7). kdSpeed <= 0 disables it.
  void setKdSpeedScheduling(float kdSpeed) { kdSpeed_ = kdSpeed; }
  // Measured ground speed in mm/s. Set every cycle before update().
  void setSpeed(float speed) { speed_ = speed; }
  float effectiveKRate();
  // Additional feedforward tilt (body degrees), e.g. to counteract dome motion. Set every cycle before update().
  void setHeadCompensation(float tilt) { headTilt_ = tilt; }
  // Use this instead of setDebug(): the base class debug output calls
  // input.present() a second time, which steps IMUControlInput's filter twice.
  void setRollDebug(bool yesno) { rollDebug_ = yesno; }

  virtual void update() override;
  float rate() { return rate_; }

protected:
  virtual bb::Result setControlOutput(float value) override;

  float shapedGoal(float dt);

  bb::IMU& imu_;
  bb::LowPassFilter rateFilter_;
  float kRate_, kff_, reduction_, center_, rate_, headTilt_;
  float kdSpeed_, speed_;
  bool rollDebug_;

  float rawGoal_, prevGoal_;
  float iBleedTau_, returnWindow_, returnTimer_, returnGoalStart_, returnIStart_;

  static const int SHAPER_HIST = 401; // 4 s at 100 Hz
  float shaperHist_[SHAPER_HIST];
  int shaperIdx_, shaperType_;
  float shaperT_;
};

#endif // BB8POSCONTROLOUTPUT_H