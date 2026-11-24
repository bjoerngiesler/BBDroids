#include "BB8Controllers.h"
#include "BB8Config.h"

BB8RollController::BB8RollController(bb::ControlInput& input, bb::ControlOutput& output, bb::IMU& imu):
  bb::PIDController(input, output),
  imu_(imu),
  rateFilter_(ROLL_RATE_FILTER_CUTOFF, 104.0),
  kRate_(0), kff_(0), reduction_(1), center_(0), rate_(0), headTilt_(0), kdSpeed_(0), speed_(0), rollDebug_(false),
  rawGoal_(0), prevGoal_(0),
  iBleedTau_(0), returnWindow_(1.5), returnTimer_(0), returnGoalStart_(0), returnIStart_(0),
  shaperIdx_(0), shaperType_(0), shaperT_(2.0) {
  for(int i=0; i<SHAPER_HIST; i++) shaperHist_[i] = 0;
}

void BB8RollController::setRollParameters(float kp, float ki, float kRate, float kff) {
  setControlParameters(kp, ki, 0.0f); // damping is done on the gyro rate, see setControlOutput()
  kRate_ = kRate;
  kff_ = kff;
}

// Input shaper on the roll goal. ZV: two halves T/2 apart, ZVD: 1/4, 1/2, 1/4 at 0, T/2, T.
// Splitting the step like this cancels the excitation of the sideways rocking mode (period T),
// which allows considerably higher kp/ki without overshoot (see sim/README.md).
float BB8RollController::shapedGoal(float dt) {
  shaperIdx_ = (shaperIdx_ + 1) % SHAPER_HIST;
  shaperHist_[shaperIdx_] = rawGoal_;
  if(shaperType_ <= 0 || dt <= 0) return rawGoal_;

  int k = (int)lroundf(shaperT_ / 2.0f / dt);
  if(k < 0) k = 0;
  if(shaperType_ == 1) {
    if(k > SHAPER_HIST-1) k = SHAPER_HIST-1;
    float past = shaperHist_[(shaperIdx_ - k + SHAPER_HIST) % SHAPER_HIST];
    return 0.5f*rawGoal_ + 0.5f*past;
  } else {
    if(2*k > SHAPER_HIST-1) k = (SHAPER_HIST-1)/2;
    float p1 = shaperHist_[(shaperIdx_ - k + SHAPER_HIST) % SHAPER_HIST];
    float p2 = shaperHist_[(shaperIdx_ - 2*k + 2*SHAPER_HIST) % SHAPER_HIST];
    return 0.25f*rawGoal_ + 0.5f*p1 + 0.25f*p2;
  }
}

void BB8RollController::update() {
  float dt = bb::Runloop::runloop.cycleTimeSeconds();

  // Goal (optionally shaped). Ramp is 0 for this controller, so setGoal() does not touch the input.
  setGoal(shapedGoal(dt));

  // Return-to-zero detection: the goal moved towards zero -> open the return window.
  if(fabs(goal_) < fabs(prevGoal_) - 1e-3f) {
    if(returnTimer_ <= 0) { returnGoalStart_ = prevGoal_; returnIStart_ = errI_; }
    returnTimer_ = returnWindow_;
  }
  prevGoal_ = goal_;
  if(returnTimer_ > 0) returnTimer_ -= dt;

  // Lean from the last cycle (goal - error). Do not call input_.present() here, that would
  // step IMUControlInput's filter a second time.
  float lean = goal_ - lastErr_;
  bool returning = returnTimer_ > 0 && fabs(goal_) < fabs(lean) - 0.5f && goal_*lean >= 0;
  if(returning && iBleedTau_ > 0) {
    // Bleed the I state towards the share that belongs to the new goal (full return -> 0).
    float target = fabs(returnGoalStart_) > 1e-6f ? returnIStart_ * goal_ / returnGoalStart_ : 0.0f;
    errI_ = target + (errI_ - target) * expf(-dt / iBleedTau_);
  }

  // Same axis convention as IMU::getFilteredPRH(), which feeds the roll input.
  float dp, dr, dh;
  if(imu_.getGyroMeasurement(dp, dr, dh)) rate_ = rateFilter_.filter(dr);
  bb::PIDController::update();
}

float BB8RollController::effectiveKRate() {
  float v = fabs(speed_);
  if(kdSpeed_ > 0 && v > kdSpeed_) return kRate_ * kdSpeed_ / v;
  return kRate_;
}

bb::Result BB8RollController::setControlOutput(float value) {
  // value is kp*err + ki*errI + controlOffset (offset is 0 for this controller)
  float kRate = effectiveKRate();
  float tilt = kff_*goal() + headTilt_ + value - kRate*rate_;
  if(rollDebug_) {
    // -lastErrD_ is the roll rate derived from the filtered angle. It should
    // roughly match the gyro rate in sign and magnitude; if the sign is
    // opposite, the damping term pushes the wrong way.
    bb::Console::console.printfBroadcast("Roll: goal %.1f head %.2f PI %.2f errI %.2f gyro rate %.1f angle rate %.1f kd %.2f tilt %.2f servo %.1f\n",
                                         goal(), headTilt_, value, errI_, rate_, -lastErrD_, kRate, tilt, center_ + reduction_*tilt);
  }
  return output_.set(center_ + reduction_*tilt);
}
