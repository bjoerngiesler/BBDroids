#include "BBIMU.h"

#include <LibBB.h>

#include <vector>
#include <limits.h>

bb::IMUControlInput::IMUControlInput(IMU& imu, IMUControlInput::ProbeType pt, bool inverse): filter_(100.0f, 100.0), imu_(imu) {
  pt_ = pt;
  bias_ = 0;
  deadband_ = 0;
  inv_ = inverse;
}

bb::Result bb::IMUControlInput::update() {
  // IMU update() is manually called in main droid step() function, so we don't do it here
  return RES_OK;
}

float bb::IMUControlInput::present() {
  float r, p, h;
  if(imu_.getFilteredRPH(r, p, h) == false) return 0.0f;
  
  float retval=0;

  switch(pt_) {
#if 0
// FIXMEEEEEEEE
  case IMU_ROLL:
    retval = filter_.filter(r-bias_);
    break;
  case IMU_PITCH:
    retval = filter_.filter(p-bias_);
    break;
#endif
  case IMU_ROLL:
    retval = filter_.filter(p-bias_);
    break;
  case IMU_PITCH:
    retval = filter_.filter(bias_-r);
    break;
  case IMU_HEADING:
    retval = filter_.filter(h-bias_);
    break;
  }

  if(fabs(retval) < fabs(deadband_)) return 0.0f;
  if(inv_) return -retval;
  return retval;
}

void bb::IMUControlInput::setFilterCutoff(float frequency) {
  filter_.setCutoff(frequency);
}

bb::IMU::IMU(uint8_t addr) {
  available_ = false;
  calR_ = calP_ = calH_ = 0.0f;
  intRunning_ = false;
  addr_ = addr;
}

bool bb::IMU::begin() {
  if(available_) return true;
  Console::console.printfBroadcast("Setting up Body IMU...");

  // Check whether we exist
  int err;
  Wire.beginTransmission(addr_);
  err = Wire.endTransmission();
  if(err != 0) {
    Console::console.printfBroadcast("Wire.endTransmission() returns error %d while detecting IMU at 0x%x\n", err, addr_);
    available_ = false;
    return false;
  }

  if(!imu_.begin_I2C(addr_)) {
    Console::console.printfBroadcast("failed!\n");
    return false;
  }

  temp_ = imu_.getTemperatureSensor();
  if(NULL == temp_) {
    Console::console.printfBroadcast("could not get temp sensor!\n");
    return false;
  }
  accel_ = imu_.getAccelerometerSensor();
  if(NULL == accel_) {
    Console::console.printfBroadcast("could not get accel sensor!\n");
    return false;
  }
  gyro_ = imu_.getGyroSensor();
  if(NULL == accel_) {
    Console::console.printfBroadcast("could not get gyro sensor!\n");
    return false;
  }

  imu_.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu_.setGyroDataRate(LSM6DS_RATE_104_HZ);

  dataRate_ = 104.0;
  Runloop::runloop.setCycleTimeMicros(1000000/104);
  
  madgwick_.begin(dataRate_);

  Console::console.printfBroadcast("ok\n");
  available_ = true;
  return true;
}

void bb::IMU::printStats(const String& prefix) {
  if(!available_ || NULL == temp_ || NULL == accel_ || NULL == gyro_)
    return;

  Serial.println(prefix);
  temp_->printSensorDetails();
  accel_->printSensorDetails();
  gyro_->printSensorDetails();
}

bool bb::IMU::update(bool block) {
  if(!available_) return false;

  int timeout = 3;
  while(!imu_.gyroscopeAvailable() && !imu_.accelerationAvailable()) {
    timeout--;
    delayMicroseconds(1);
    if(timeout < 0 && block==false) {
      //Console::console.printfBroadcast("No gyro or accel data available\n");
      return false;
    }
  }
  
  imu_.readGyroscope(lastR_, lastP_, lastH_);
  imu_.readAcceleration(lastX_, lastY_, lastZ_);

  madgwick_.updateIMU(lastR_ + calR_, lastP_ + calP_, lastH_ + calH_, lastX_, lastY_, lastZ_);

  return true;
}

bool bb::IMU::getFilteredRPH(float &r, float &p, float &h) {
  if(!available_) return false;

#if 0 // FIXME
  r = madgwick_.getRoll();
  p = madgwick_.getPitch();
#else
  r = madgwick_.getPitch();
  p = madgwick_.getRoll();
#endif
  h = madgwick_.getYaw();
  return true;
}

bool bb::IMU::getGyroMeasurement(float& r, float& p, float& h, bool calibrated) {
  if(!available_) return false;

  r = lastR_; p = lastP_; h = lastH_;
  if(calibrated) {
    r += calR_;
    p += calP_;
    h += calH_;
  }
  
  return true;
}

bool bb::IMU::getAccelMeasurement(float &x, float &y, float &z) {
  if(!available_) return false;

  x = lastX_;
  y = lastY_;
  z = lastZ_;
  
  return true;
}


bool bb::IMU::calibrateGyro(ConsoleStream *stream, int milliseconds, int step) {
  if(!available_) return false;

  double avgTemp = 0.0, avgR = 0.0, avgP = 0.0, avgH = 0.0;
  int count = 0;

  sensors_event_t t;

  for(int ms = milliseconds; ms>0; ms -= step, count++) {
    float r, p, h;
    if(imu_.gyroscopeAvailable()) imu_.readGyroscope(r, p, h);

    temp_->getEvent(&t);

    avgTemp += t.temperature;
    avgR += r;
    avgP += p;
    avgH += h;

    delay(step);
  }
  
  avgTemp /= count;
  avgR /= count;
  avgP /= count;
  avgH /= count;

  if(stream) {
    stream->printf("Gyro calib finished (%d cycles, avg temp %f°C)\n", count, avgTemp);
    stream->printf("R=%.6f P=%.6f H=%.6f\n", avgR, avgP, avgH);
  }

  calR_ = -avgR; calP_ = -avgP; calH_ = -avgH;

  return true;
}

bb::IMUState bb::IMU::getIMUState() {
  bb::IMUState imuState;
  if(!available_) {
    imuState.errorState = ERROR_NOT_PRESENT;
    return imuState;
  }

  imuState.errorState = ERROR_OK;
  imuState.r = madgwick_.getRoll();
  imuState.p = madgwick_.getPitch();
  imuState.h = madgwick_.getYaw();
  imuState.dr = lastR_;
  imuState.dp = lastP_;
  imuState.dh = lastH_;
  imuState.ax = lastX_;
  imuState.ay = lastY_;
  imuState.az = lastZ_;

  return imuState;
}
