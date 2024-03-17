#include "DOIMU.h"

#include <LibBB.h>

#include <vector>
#include <limits.h>

using namespace bb;

DOIMU DOIMU::imu;

DOIMUControlInput::DOIMUControlInput(DOIMUControlInput::ProbeType pt): filter_(2.0f) {
  pt_ = pt;
  bias_ = 0;
  deadband_ = 0;
}

bb::Result DOIMUControlInput::update() {
  // IMU update() is manually called in main droid step() function, so we don't do it here
  return RES_OK;
}

float DOIMUControlInput::present() {
  float r, p, h;
  if(DOIMU::imu.getFilteredRPH(r, p, h) == false) return 0.0f;
  
  float retval;

  switch(pt_) {
  case IMU_ROLL:
    retval = filter_.filter(r-bias_);
    break;
  case IMU_PITCH:
    retval = filter_.filter(p-bias_);
    break;
  case IMU_HEADING:
    retval = filter_.filter(h-bias_);
    break;
  }

  if(fabs(retval) < fabs(deadband_)) return 0.0f;
  return retval;

  return 0.0f;
}

void DOIMUControlInput::setFilterFrequency(float frequency) {
  filter_.setSampleFrequency(frequency);
}

DOIMU::DOIMU() {
  available_ = false;
  calR_ = calP_ = calH_ = 0.0f;
  intRunning_ = false;
}

bool DOIMU::begin() {
  if(available_) return true;
  Console::console.printfBroadcast("Setting up Body IMU...\n");

  // Check whether we exist
  int err;
  Wire.beginTransmission(IMU_ADDR);
  err = Wire.endTransmission();
  if(err != 0) {
    bb::Console::console.printfBroadcast("Wire.endTransmission() returns error %d while detecting IMU at 0x%x\n", err, IMU_ADDR, HEX);
    available_ = false;
    return false;
  }

  if(!imu_.begin_I2C()) {
    Serial.println("failed!");
    return false;
  }

  temp_ = imu_.getTemperatureSensor();
  if(NULL == temp_) {
    Serial.println("could not get temp sensor!");
    return false;
  }
  accel_ = imu_.getAccelerometerSensor();
  if(NULL == accel_) {
    Serial.println("could not get accel sensor!");
    return false;
  }
  gyro_ = imu_.getGyroSensor();
  if(NULL == accel_) {
    Serial.println("could not get gyro sensor!");
    return false;
  }

  lsm6ds_data_rate_t dataRate = imu_.getGyroDataRate();
  if(dataRate == LSM6DS_RATE_104_HZ) {
    dataRate_ = 104.0;
    Runloop::runloop.setCycleTimeMicros(1000000/104);
    Serial.print("data rate of 104Hz... ");
  } else {
    dataRate_ = 1e6/Runloop::runloop.cycleTimeMicros();
    Serial.print(String("unknown data rate ") + dataRate);
  }
  
  madgwick_.begin(dataRate_);

  Serial.println("ok");
  available_ = true;
  return true;
}

void DOIMU::printStats(const String& prefix) {
  if(!available_ || NULL == temp_ || NULL == accel_ || NULL == gyro_)
    return;

  Serial.println(prefix);
  temp_->printSensorDetails();
  accel_->printSensorDetails();
  gyro_->printSensorDetails();
}

bool DOIMU::update() {
  if(!available_) return false;

  if(!imu_.gyroscopeAvailable() || !imu_.accelerationAvailable()) return false;
  
  imu_.readGyroscope(lastR_, lastP_, lastH_);
  imu_.readAcceleration(lastX_, lastY_, lastZ_);

  madgwick_.updateIMU(lastR_ + calR_, lastP_ + calP_, lastH_ + calH_, lastX_, lastY_, lastZ_);

  return true;
}

bool DOIMU::getFilteredRPH(float &r, float &p, float &h) {
  if(!available_) return false;

  r = madgwick_.getRoll();
  p = madgwick_.getPitch();
  h = madgwick_.getYaw();
  return true;
}

bool DOIMU::getGyroMeasurement(float& r, float& p, float& h, bool calibrated) {
  if(!available_) return false;

  r = lastR_; p = lastP_; h = lastH_;
  if(calibrated) {
    r += calR_;
    p += calP_;
    h += calH_;
  }
  
  return true;
}

bool DOIMU::getAccelMeasurement(float &x, float &y, float &z, uint32_t &t) {
  if(!available_) return false;

  sensors_event_t a;
  accel_->getEvent(&a);
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
  t = a.timestamp;
  
  return true;
}


bool DOIMU::calibrateGyro(ConsoleStream *stream, int milliseconds, int step) {
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

bb::IMUState DOIMU::getIMUState() {
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
