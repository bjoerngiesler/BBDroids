#if !defined(BB8IMU_H)
#define BB8IMU_H

#include <math.h>
#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>

#include "BB8Config.h"

class BB8BodyIMU {
public:
  static BB8BodyIMU imu;

  bool begin();
  bool begin(int cycletime);

  bool available() { return available_; }

  bool calibrateGyro(int milliseconds = 2000, int step = 10);

  bool integrateGyroMeasurement(bool reset = false);
  int getIntegratedGyroMeasurement(float& r, float& p, float& h);

  bool getGyroMeasurement(float& r, float& p, float& h, int32_t& timestamp, bool calibrated=true);
  bool getAccelMeasurement(float& x, float& y, float& z, int32_t& timestamp);

  virtual bool step(bool outputForPlot=false);
  bool getFilteredRPH(float& r, float& p, float& h);

  bool printGyroMeasurementForPlot();

  void printStats(const arduino::String& prefix = "");
private:  
  BB8BodyIMU();

  Madgwick madgwick_;
  bool available_;
  Adafruit_ISM330DHCX imu_;
  Adafruit_Sensor *temp_, *accel_, *gyro_;
  float calR_, calP_, calH_;
  float intR_, intP_, intH_;
  int intNum_;
  int32_t intLastTS_;
  bool intRunning_ = false;
};

#endif // BB8IMU_H
