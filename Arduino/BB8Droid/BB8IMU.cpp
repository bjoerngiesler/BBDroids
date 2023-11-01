#include "BB8IMU.h"

#include <LibBB.h>

#include <vector>
#include <limits.h>

using namespace bb;

BB8IMU BB8IMU::imu;

BB8IMUControlInput::BB8IMUControlInput(BB8IMUControlInput::ProbeType pt) {
  pt_ = pt;
}

bb::Result BB8IMUControlInput::update() {
  if(BB8IMU::imu.update() == true) return RES_OK;

  return RES_CMD_FAILURE;
}

float BB8IMUControlInput::present() {
  float r, p, h;
  if(BB8IMU::imu.getFilteredRPH(r, p, h) == false) return 0.0f;
  
  switch(pt_) {
  case IMU_ROLL:
    return r;
    break;
  case IMU_PITCH:
    return p;
    break;
  case IMU_HEADING:
    return h;
    break;
  }

  return 0.0f;
}

BB8IMU::BB8IMU() {
  available_ = false;
  calR_ = calP_ = calH_ = 0.0f;
  intRunning_ = false;
}

bool BB8IMU::begin() {
  if(available_) return true;
  Serial.print("Setting up Body IMU... ");
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
    Runloop::runloop.setCycleTime(1000000/104);
    madgwick_.begin(104);
    Serial.print("data rate of 104Hz... ");
  } else {
    madgwick_.begin(1000000/Runloop::runloop.cycleTime());
    Serial.print(String("unknown data rate ") + dataRate);
  }

  Serial.println("ok");
  available_ = true;
  return true;
}

void BB8IMU::printStats(const String& prefix) {
  if(!available_ || NULL == temp_ || NULL == accel_ || NULL == gyro_)
    return;

  Serial.println(prefix);
  temp_->printSensorDetails();
  accel_->printSensorDetails();
  gyro_->printSensorDetails();
}

bool BB8IMU::update() {
  if(!available_) return false;

  //if(!imu_.gyroscopeAvailable() || !imu_.accelerationAvailable()) return false;
  
  imu_.readGyroscope(lastR_, lastP_, lastH_);
  imu_.readAcceleration(lastX_, lastY_, lastZ_);

  madgwick_.updateIMU(lastR_ + calR_, lastP_ + calP_, lastH_ + calH_, lastX_, lastY_, lastZ_);

  return true;
}

bool BB8IMU::getFilteredRPH(float &r, float &p, float &h) {
  if(!available_) return false;

  r = madgwick_.getRoll();
  p = madgwick_.getPitch();
  h = madgwick_.getYaw();
  return true;
}

bool BB8IMU::integrateGyroMeasurement(bool reset) {
  if(!available_) return false;

  sensors_event_t g;
  gyro_->getEvent(&g);
  if(intRunning_ == false || reset == true) {
    intR_ = g.gyro.roll; 
    intP_ = g.gyro.pitch;
    intH_ = g.gyro.heading;
    intLastTS_ = g.timestamp;
    intRunning_ = true;
    intNum_ = 1;
    return true;
  }

  float dt = 0;
  if(g.timestamp < intLastTS_) { // wrap
    dt = (float)(((INT_MAX - intLastTS_) + g.timestamp)) / 1000.0f;
    Serial.print("wrap dt:"); Serial.println(dt, 10);
  } else {
    dt = (float)(g.timestamp - intLastTS_) / 1000.0f;
  }

  intR_ += dt*(g.gyro.roll + calR_);
  intP_ += dt*(g.gyro.pitch + calP_);
  intH_ += dt*(g.gyro.heading + calH_); 

  intLastTS_ = g.timestamp;
  intNum_++;

  return true;
}

int BB8IMU::getIntegratedGyroMeasurement(float& r, float& p, float& h) {
  if(!available_) return false;

  r = intR_*180.0/M_PI; p = intP_*180.0/M_PI; h = intH_*180.0/M_PI;
  return intNum_;
}

bool BB8IMU::getGyroMeasurement(float& r, float& p, float& h, bool calibrated) {
  if(!available_) return false;

  r = lastR_; p = lastP_; h = lastH_;
  if(calibrated) {
    r += calR_;
    p += calP_;
    h += calH_;
  }
  
  return true;
}

bool BB8IMU::getAccelMeasurement(float &x, float &y, float &z, int32_t &t) {
  if(!available_) return false;

  sensors_event_t a;
  accel_->getEvent(&a);
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
  t = a.timestamp;
  
  return true;
}


bool BB8IMU::calibrateGyro(ConsoleStream *stream, int milliseconds, int step) {
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

    if(stream) stream->println(String("R=") + String(r, 6) + " P=" + String(p, 6) + " H=" + String(h, 6));

    delay(step);
  }
  
  avgTemp /= count;
  avgR /= count;
  avgP /= count;
  avgH /= count;

  if(stream) {
    stream->print(String("Gyro calib finished (") + count + "cycles, avg temp " + avgTemp + "°C). ");
    stream->println(String("R=") + String(avgR, 6) + " P=" + String(avgP, 6) + " H=" + String(avgH, 6));
  }

  calR_ = -avgR; calP_ = -avgP; calH_ = -avgH;

  return true;
}

bb::IMUState BB8IMU::getIMUState() {
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
