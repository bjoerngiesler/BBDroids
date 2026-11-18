#include "BB8Controllers.h"
#include "BB8Config.h"

BB8RollController::BB8RollController(bb::ControlInput& input, bb::ControlOutput& output, bb::IMU& imu):
  bb::PIDController(input, output),
  imu_(imu),
  rateFilter_(ROLL_RATE_FILTER_CUTOFF, 104.0),
  kRate_(0), kff_(0), reduction_(1), center_(0), rate_(0), headTilt_(0), rollDebug_(false) {
}

void BB8RollController::setRollParameters(float kp, float ki, float kRate, float kff) {
  setControlParameters(kp, ki, 0.0f); // damping is done on the gyro rate, see setControlOutput()
  kRate_ = kRate;
  kff_ = kff;
}

void BB8RollController::update() {
  // Same axis convention as IMU::getFilteredPRH(), which feeds the roll input.
  float dp, dr, dh;
  if(imu_.getGyroMeasurement(dp, dr, dh)) rate_ = rateFilter_.filter(dr);
  bb::PIDController::update();
}

bb::Result BB8RollController::setControlOutput(float value) {
  // value is kp*err + ki*errI + controlOffset (offset is 0 for this controller)
  float tilt = kff_*goal() + headTilt_ + value - kRate_*rate_;
  if(rollDebug_) {
    // -lastErrD_ is the roll rate derived from the filtered angle. It should
    // roughly match the gyro rate in sign and magnitude; if the sign is
    // opposite, the damping term pushes the wrong way.
    bb::Console::console.printfBroadcast("Roll: goal %.1f head %.2f PI %.2f gyro rate %.1f angle rate %.1f tilt %.2f servo %.1f\n",
                                         goal(), headTilt_, value, rate_, -lastErrD_, tilt, center_ + reduction_*tilt);
  }
  return output_.set(center_ + reduction_*tilt);
}
