#if !defined(BB8IMU_H)
#define BB8IMU_H

#include <LibBB.h>

#include <math.h>
#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>

#include "BB8Config.h"

using namespace bb;

class BB8IMUControlInput: public bb::ControlInput {
public:
  typedef enum {
    IMU_ROLL,
    IMU_PITCH,
    IMU_HEADING
  } ProbeType;

  BB8IMUControlInput(ProbeType pt);
  float present();
  Result update();
protected:
  ProbeType pt_;
};

class BB8IMU {
public:
  static BB8IMU imu;

  bool begin();

  bool available() { return available_; }

  bool calibrateGyro(ConsoleStream *stream=NULL, int milliseconds = 2000, int step = 10);

  bool integrateGyroMeasurement(bool reset = false);
  int getIntegratedGyroMeasurement(float& r, float& p, float& h);

  bool getGyroMeasurement(float& r, float& p, float& h, bool calibrated=true);
  bool getAccelMeasurement(float& x, float& y, float& z, int32_t& timestamp);

  virtual bool update();
  bool getFilteredRPH(float& r, float& p, float& h);

  IMUState getIMUState();
  void printStats(const arduino::String& prefix = "");
private:  
  BB8IMU();

  Madgwick madgwick_;
  bool available_;
  Adafruit_ISM330DHCX imu_;
  Adafruit_Sensor *temp_, *accel_, *gyro_;
  float calR_, calP_, calH_;
  float lastR_, lastP_, lastH_;
  float intR_, intP_, intH_;
  float lastX_, lastY_, lastZ_;
  int intNum_;
  int32_t intLastTS_;
  bool intRunning_ = false;
};

#endif // BB8IMU_H
