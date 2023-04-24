#include "BB8IMU.h"

#include <LibBB.h>

#include <vector>
#include <limits.h>

using namespace bb;

BB8BodyIMU BB8BodyIMU::imu;
//BB8IMU BB8IMU::dome;

BB8BodyIMU::BB8BodyIMU() {
  available_ = false;
  calR_ = calP_ = calH_ = 0.0f;
  intRunning_ = false;
}

bool BB8BodyIMU::begin() {
  if(available_) return true;
  Serial.print("Setting up Body IMU... ");
  if(!imu_.begin_I2C()) {
    Serial.println("failed!");
    return false;
  }

  temp_ = imu_.getTemperatureSensor();
  if(NULL == temp_) {
    Serial.println("could not get temperature sensor pointer!");
    return false;
  }
  accel_ = imu_.getAccelerometerSensor();
  if(NULL == accel_) {
    Serial.println("could not get accel sensor pointer!");
    return false;
  }
  gyro_ = imu_.getGyroSensor();
  if(NULL == accel_) {
    Serial.println("could not get gyro sensor pointer!");
    return false;
  }

  madgwick_.begin(1000/Runloop::runloop.cycleTime());

  Serial.println("ok");
  available_ = true;
  return true;
}

void BB8BodyIMU::printStats(const String& prefix) {
  if(!available_ || NULL == temp_ || NULL == accel_ || NULL == gyro_)
    return;

  Serial.println(prefix);
  temp_->printSensorDetails();
  accel_->printSensorDetails();
  gyro_->printSensorDetails();
}

bool BB8BodyIMU::update() {
  if(!available_) return false;

  float x, y, z;

  if(imu_.gyroscopeAvailable()) imu_.readGyroscope(lastR_, lastP_, lastH_);
  if(imu_.accelerationAvailable()) imu_.readAcceleration(x, y, z);

  madgwick_.updateIMU(lastR_ + calR_, lastP_ + calP_, lastH_ + calH_, x, y, z);

  return true;
}

bool BB8BodyIMU::getFilteredRPH(float &r, float &p, float &h) {
  if(!available_) return false;

  r = madgwick_.getRoll();
  p = madgwick_.getPitch();
  h = madgwick_.getYaw();
  return true;
}

bool BB8BodyIMU::integrateGyroMeasurement(bool reset) {
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

int BB8BodyIMU::getIntegratedGyroMeasurement(float& r, float& p, float& h) {
  if(!available_) return false;

  r = intR_*180.0/M_PI; p = intP_*180.0/M_PI; h = intH_*180.0/M_PI;
  return intNum_;
}

bool BB8BodyIMU::getGyroMeasurement(float& r, float& p, float& h, bool calibrated) {
  if(!available_) return false;

  r = lastR_; p = lastP_; h = lastH_;
  if(calibrated) {
    r += calR_;
    p += calP_;
    h += calH_;
  }
  
  return true;
}

bool BB8BodyIMU::getAccelMeasurement(float &x, float &y, float &z, int32_t &t) {
  if(!available_) return false;

  sensors_event_t a;
  accel_->getEvent(&a);
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
  t = a.timestamp;
  
  return true;
}


bool BB8BodyIMU::calibrateGyro(ConsoleStream *stream, int milliseconds, int step) {
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
    stream->print(String("Gyro calibration finished (") + count + "cycles at average temp " + avgTemp + "°C). ");
    stream->println(String("Calib values (rad/s): R=") + String(avgR, 6) + " P=" + String(avgP, 6) + " H=" + String(avgH, 6));
  }

  calR_ = -avgR; calP_ = -avgP; calH_ = -avgH;

  return true;
}

bool BB8BodyIMU::printGyroMeasurementForPlot() {
  float r, p, h;
  int32_t t;
  getGyroMeasurement(r, p, h, false);
  Serial.print("UR:"); Serial.print(r, 10);
  Serial.print(",UP:"); Serial.print(p, 10);
  Serial.print(",UH:"); Serial.print(h, 10);
  getGyroMeasurement(r, p, h, t);
  Serial.print(",R:"); Serial.print(r, 10);
  Serial.print(",P:"); Serial.print(p, 10);
  Serial.print(",H:"); Serial.print(h, 10);
  Serial.println();
  return true;
}