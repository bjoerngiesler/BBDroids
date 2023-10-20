#if !defined(BB8BATTSTATUS_H)
#define BB8BATTSTATUS_H

class BB8BattStatus {
public:
  static BB8BattStatus batt;

  typedef enum {
    BATT_1    = 0,
    BATT_2    = 1,
    BATT_BOTH = 2
  } Battery;

  bool begin();
  bool available(Battery batt = BATT_BOTH);
  bool updateVoltage(Battery batt = BATT_BOTH);
  bool updateCurrent(Battery batt = BATT_BOTH);

  // Returns sum of both currents if called with BATT_BOTH. Returns -1 if error.
  float current(Battery batt); 
  // Returns minimum of both voltages if called with BATT_BOTH. Returns -1 if error.
  float voltage(Battery batt);

private:
  BB8BattStatus();
  typedef struct {
    bool available;
    float voltage; float current;
  } BattStatus;
  BattStatus status_[2];
};

#endif