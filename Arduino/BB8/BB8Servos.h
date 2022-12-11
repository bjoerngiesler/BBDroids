#if !defined(BB8SERVOS_H)
#define BB8SERVOS_H

#include <DynamixelShield.h>

class BB8Servos {
  public:
  static BB8Servos servos;
  bool begin();
  bool available() { return available_; }
  void printStatus();
  bool moveAllServosToOrigin();
  bool setGoalPosition(uint8_t servo, float goal);
  float getPresentPosition(uint8_t servo);
  bool switchTorque(uint8_t servo, bool onoff);
  bool isTorqueOn(uint8_t servo);

  protected:
  BB8Servos();
  DynamixelShield dxl_;
  bool available_;
};

#endif // BB8SERVOS_H