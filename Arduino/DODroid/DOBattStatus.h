#if !defined(DOBATTSTATUS_H)
#define DOBATTSTATUS_H

#include <LibBB.h>

class DOBattStatus {
public:
  static DOBattStatus batt;

  bool begin();
  bool available();
  bool updateVoltage();
  bool updateCurrent();

  // Returns -1 if error.
  float current(); 
  // Returns -1 if error.
  float voltage();

  bb::BatteryState getBatteryState();

protected:
  DOBattStatus();
  bool available_;
  float voltage_; 
  float current_;
};

#endif