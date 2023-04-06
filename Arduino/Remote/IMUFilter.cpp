#include "IMUFilter.h"

#include <Arduino_LSM6DSOX.h>
#include <limits.h>
#include <math.h>

IMUFilter IMUFilter::imu;

#define FILTERSIZE 5

IMUFilter::IMUFilter() {
  available_ = false;
}

bool IMUFilter::begin() {
  if(IMU.begin()) {
    available_ = true;
    Serial.print("Gyroscope available, sample rate: ");
    Serial.println(IMU.gyroscopeSampleRate());
    update();
    return true;
  } 

  Serial.println("Gyroscope not available");
  available_ = false;
  return true;
}

void IMUFilter::update() {
  if(!available_) return;
  float dp, dy, dr;

  if(!IMU.gyroscopeAvailable()) return;
  
  IMU.readGyroscope(dr, dp, dy);

  if(!initialized_) {
    lastMillis_ = millis();
    r_ = 0; p_ = 0; y_ = 0;
    initialized_ = true;
  } else {
    if(isnan(dp)) Serial.println("dp is NAN");
    if(isnan(dy)) Serial.println("dy is NAN");
    if(isnan(dr)) Serial.println("dr is NAN");

    if(!isnan(dp) && !isnan(dy) && !isnan(dr)) {
      unsigned int m = millis();
      float dt;
      if(m < lastMillis_) {
        dt = ((float)(UINT_MAX - lastMillis_ + m)) / 1000.0f;
      } else {
        dt = ((float)(m - lastMillis_)) / 1000.0f;
      }
      lastMillis_ = m;

// #define MAXANGLE M_PI
#define MAXANGLE 180.0

      r_ += dt * dr;
      if(r_ < -MAXANGLE) r_ += 2*MAXANGLE;
      else if(r_ > MAXANGLE) r_ -= 2*MAXANGLE;
      p_ += dt * dp;
      if(p_ < -MAXANGLE) p_ += 2*MAXANGLE;
      else if(p_ > MAXANGLE) p_ -= 2*MAXANGLE;
      y_ += dt * dy;
      if(y_ < -MAXANGLE) y_ += 2*MAXANGLE;
      else if(y_ > MAXANGLE) y_ -= 2*MAXANGLE;
    }
  }

  while(rolls_.size() > FILTERSIZE) rolls_.erase(rolls_.begin()); 
  while(pitches_.size() > FILTERSIZE) pitches_.erase(pitches_.begin()); 
  while(yaws_.size() > FILTERSIZE) yaws_.erase(yaws_.begin()); 
  rolls_.push_back(r_);
  pitches_.push_back(p_);
  yaws_.push_back(y_);
}

void IMUFilter::getFilteredEulerAngles(float& roll, float &pitch, float &yaw) {
  roll = 0;
  for(size_t i=0; i<rolls_.size(); i++) roll+=rolls_[i];
  roll /= rolls_.size();
  pitch = 0;
  for(size_t i=0; i<pitches_.size(); i++) pitch+=pitches_[i];
  pitch /= pitches_.size();
  yaw = 0;
  for(size_t i=0; i<yaws_.size(); i++) yaw+=yaws_[i];
  yaw /= yaws_.size();
}

void IMUFilter::getRawEulerAngles(float& roll, float &pitch, float &yaw) {
  roll = r_; pitch = p_; yaw = y_;
}