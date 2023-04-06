#include "actuator.h"
#if !defined(BB8SERVOS_H)
#define BB8SERVOS_H

#include <LibBB.h>
#include <DynamixelShield.h>
#include <vector>

using namespace bb;

class BB8ServoPower {
public:
  static BB8ServoPower power;

  virtual Result initialize();
  virtual bool isOn();
  virtual Result switchOnOff(bool onoff);

protected:
  bool requestFrom(uint8_t addr, uint8_t reg, uint8_t& byte);
  void writeTo(uint8_t addr, uint8_t reg, uint8_t byte);
};

class BB8Servos: public Subsystem {
  public:
  static BB8Servos servos;

	virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  Result handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String>& words, ConsoleStream *stream);

  Result runServoTest(ConsoleStream *stream, int id);
  
  void printStatus(ConsoleStream* stream, int id);
  Result moveAllServosToOrigin(bool hard = false);
  bool setSpeed(uint8_t servo, float speed);
  bool setSetpoint(uint8_t servo, float setpoint);
  bool setPosition(uint8_t servo, float goal);
  float getPresentPosition(uint8_t servo);
  Result switchTorque(uint8_t servo, bool onoff);
  Result switchTorqueAll(bool onoff);
  bool isTorqueOn(uint8_t servo);

protected:
  BB8Servos();
  DynamixelShield dxl_;

  typedef struct {
    int id;
    bool available;
    float speed;
    float setpoint;
    float current;
  } Servo;

  Servo servos_[4];
};

#endif // BB8SERVOS_H