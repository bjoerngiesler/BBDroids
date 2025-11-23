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
  float p, r, h;
  if(imu_.getFilteredPRH(p, r, h) == false) return 0.0f;
  
  float retval=0;

  switch(pt_) {
  case IMU_PITCH:
    retval = filter_.filter(p-bias_);
    break;
  case IMU_ROLL:
    retval = filter_.filter(r-bias_);
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

bb::IMU::IMU() {
  available_ = false;
  calP_ = calR_ = calH_ = 0.0f;
  calX_ = calY_ = calZ_ = 0.0f;
  intRunning_ = false;
  rot_ = ROTATE_0;
}

bool bb::IMU::begin(uint8_t addr) {
  if(available_) return true;

  addr_ = addr;

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
    Console::console.printfBroadcast("IMU setup at 0x%x failed!\n", addr_);
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
  Runloop::runloop.setCycleTimeMicros(1000000/dataRate_);
  madgwick_.begin(dataRate_);

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
  if(block == true) {
    while(!imu_.gyroscopeAvailable() && !imu_.accelerationAvailable()) {
      if(timeout < 0) {
        return false;
      }
      timeout--;
      delayMicroseconds(1);
    }
  }
  
  imu_.readGyroscope(lastP_, lastR_, lastH_);
  imu_.readAcceleration(lastX_, lastY_, lastZ_);

  madgwick_.updateIMU(lastP_+calP_, lastR_+calR_, lastH_+calH_, lastX_, lastY_, lastZ_);

  return true;
}

bool bb::IMU::getFilteredPRH(float &p, float &r, float &h) {
  if(!available_) return false;

  // Madgwick filter switches axes
  r = madgwick_.getPitch();
  p = madgwick_.getRoll();
  h = madgwick_.getYaw();
  float temp;

  switch(rot_) {
    case ROTATE_90: temp = r; r = p; p = -temp; break;
    case ROTATE_180: r = -r; p = -p; break;
    case ROTATE_270: temp = r; r = -p; p = temp; break;    
    case ROTATE_0: 
    default:
    break;
  }

  if(isnan(p) || isnan(r) || isnan(h)) {
    Console::console.printfBroadcast("IMU returns NAN!!!\n");
    available_ = false;
    p = 0; r = 0; h = 0;
  }

  return true;
}

bool bb::IMU::getGyroMeasurement(float& p, float& r, float& h, bool calibrated) {
  if(!available_) return false;

  p = lastP_; r = lastR_; h = lastH_;
  if(calibrated) {
    p += calP_;
    r += calR_;
    h += calH_;
  }
  float temp;

  switch(rot_) {
    case ROTATE_90: temp = r; r = p; p = -temp; break;
    case ROTATE_180: r = -r; p = -p; break;
    case ROTATE_270: temp = r; r = -p; p = temp; break;    
    case ROTATE_0: 
    default:
    break;
  }

  return true;
}

bool bb::IMU::getAccelMeasurement(float &x, float &y, float &z, bool calibrated) {
  if(!available_) return false;

  x = lastX_; y = lastY_; z = lastZ_;
  if(calibrated) {
    x += calX_; y += calY_; z += calZ_;
  }
  float temp;
  
  switch(rot_) {
    case ROTATE_90: temp = x; x = y; y = -temp; break;
    case ROTATE_180: x = -x; y = -y; break;
    case ROTATE_270: temp = x; x = -y; y = temp; break;    
    case ROTATE_0: 
    default:
    break;
  }

  return true;
}

bool bb::IMU::getGravCorrectedAccel(float& ax, float& ay, float& az) {
  if(!available_) return false;
  float p, r, h, gravx, gravy, gravz;
  getAccelMeasurement(ax, ay, az);
  getFilteredPRH(p, r, h);
  bb::transformVector(0, 0, 1, p, r, h, gravx, gravy, gravz, true);
  ax -= gravx;
  ay -= gravy;
  az -= gravz;
  return true;
}


bool bb::IMU::calibrate(ConsoleStream *stream, int milliseconds, int step) {
  if(!available_) return false;

  double avgTemp = 0.0, avgR = 0.0, avgP = 0.0, avgH = 0.0, avgX = 0.0, avgY = 0.0, avgZ = 0.0;
  int count = 0;

  sensors_event_t t;

  for(int ms = milliseconds; ms>0; ms -= step, count++) {
    float r, p, h, x, y, z;
    if(imu_.gyroscopeAvailable()) imu_.readGyroscope(p, r, h);
    if(imu_.accelerationAvailable()) imu_.readAcceleration(x, y, z);

    temp_->getEvent(&t);

    avgTemp += t.temperature;
    avgP += p; avgR += r; avgH += h;
    avgX += x; avgY += y; avgZ += z;

    delay(step);
  }
  
  avgTemp /= count;
  avgP /= count; avgR /= count; avgH /= count;
  avgX /= count; avgY /= count; avgZ /= count;
  avgZ -= 1.0;

  if(stream) {
    stream->printf("Calib finished (%d cycles, avg temp %f°C)\n", count, avgTemp);
    stream->printf("P=%.6f R=%.6f H=%.6f X=%.6f Y=%.6f Z=%.6f\n", avgP, avgR, avgH, avgX, avgY, avgZ);
  }

  calP_ = -avgP; calR_ = -avgR; calH_ = -avgH;
  calX_ = -avgX; calY_ = -avgY; calZ_ = -avgZ;

  return true;
}

bb::IMUState bb::IMU::getIMUState() {
  bb::IMUState imuState;
  if(!available_) {
    imuState.errorState = ERROR_NOT_PRESENT;
    return imuState;
  }

  imuState.errorState = ERROR_OK;
  imuState.p = madgwick_.getPitch();
  imuState.r = madgwick_.getRoll();
  imuState.h = madgwick_.getYaw();
  imuState.dp = lastP_;
  imuState.dr = lastR_;
  imuState.dh = lastH_;
  imuState.ax = lastX_;
  imuState.ay = lastY_;
  imuState.az = lastZ_;

  return imuState;
}
