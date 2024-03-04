#if !defined(DOIMU_H)
#define DOIMU_H

#include <LibBB.h>

#include <math.h>
#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>

#include "DOConfig.h"

using namespace bb;

class DOIMUControlInput: public bb::ControlInput {
public:
  typedef enum {
    IMU_ROLL,
    IMU_PITCH,
    IMU_HEADING
  } ProbeType;

  DOIMUControlInput(ProbeType pt);
  float present();
  Result update();

  void setFilterFrequency(float frequency);
  void setBias(float bias) { bias_ = bias; }
  void setDeadband(float deadband) { deadband_ = deadband; }
protected:
  ProbeType pt_;
  bb::LowPassFilter filter_;
  float bias_, deadband_;
};


class DOIMU {
public:
  static DOIMU imu;

  bool begin();

  bool available() { return available_; }

  bool calibrateGyro(ConsoleStream *stream=NULL, int milliseconds = 2000, int step = 10);

  bool getGyroMeasurement(float& dr, float& dp, float& dh, bool calibrated=true);
  bool getAccelMeasurement(float& ax, float& ay, float& az, uint32_t& timestamp);

  virtual bool update();
  bool getFilteredRPH(float& r, float& p, float& h);

  IMUState getIMUState();
  void printStats(const arduino::String& prefix = "");
private:  
  DOIMU();

  Madgwick madgwick_;
  bool available_;
  Adafruit_ISM330DHCX imu_;
  Adafruit_Sensor *temp_, *accel_, *gyro_;
  float calR_, calP_, calH_;
  float lastR_, lastP_, lastH_;
  float lastX_, lastY_, lastZ_;
  float intR_, intP_, intH_;
  int intNum_;
  int32_t intLastTS_;
  bool intRunning_ = false;
};

#endif // DOIMU_H
