#if !defined(BBIMU_H)
#define BBIMU_H

#include "BBConsole.h"
#include "BBControllers.h"
#include "BBLowPassFilter.h"
#include "BBPacket.h"

#include <math.h>
#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>

namespace bb {

class IMU;

class IMUControlInput: public bb::ControlInput {
public:
  typedef enum {
    IMU_ROLL,
    IMU_PITCH,
    IMU_HEADING
  } ProbeType;

  IMUControlInput(IMU& imu, ProbeType pt);
  float present();
  Result update();

  void setFilterCutoff(float frequency);
  void setBias(float bias) { bias_ = bias; }
  void setDeadband(float deadband) { deadband_ = deadband; }
protected:
  ProbeType pt_;
  bb::LowPassFilter filter_;
  float bias_, deadband_;
  IMU& imu_;
};


class IMU {
public:
  IMU(uint8_t addr);

  bool begin();

  bool available() { return available_; }

  bool calibrateGyro(ConsoleStream *stream=NULL, int milliseconds = 2000, int step = 10);

  bool getGyroMeasurement(float& dr, float& dp, float& dh, bool calibrated=true);
  bool getAccelMeasurement(float& ax, float& ay, float& az);
  float dataRate() { return dataRate_; }

  virtual bool update(bool block=false);
  bool getFilteredRPH(float& r, float& p, float& h);

  IMUState getIMUState();
  void printStats(const arduino::String& prefix = "");
private:  

  Madgwick madgwick_;
  bool available_;
  Adafruit_ISM330DHCX imu_;
  Adafruit_Sensor *temp_, *accel_, *gyro_;
  float calR_, calP_, calH_;
  float lastR_, lastP_, lastH_;
  float lastX_, lastY_, lastZ_;
  float intR_, intP_, intH_;
  int intNum_;
  float dataRate_;
  int32_t intLastTS_;
  bool intRunning_ = false;
  uint8_t addr_;
};

}; // namespace bb
#endif // BBIMU_H
