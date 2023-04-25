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
  typedef enum {
    UNIT_DEGREES,
    UNIT_RAW
  } Unit;

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
  
  bool setGoal(uint8_t id, float goal, Unit unit=UNIT_DEGREES);
  float goal(uint8_t id, Unit unit=UNIT_DEGREES);
  float present(uint8_t id, Unit unit=UNIT_DEGREES);
  
  Result switchTorque(uint8_t id, bool onoff);
  Result switchTorqueAll(bool onoff);
  bool isTorqueOn(uint8_t servo);

protected:
  BB8Servos();
  DynamixelShield dxl_;

  typedef struct {
    uint32_t goal;
    uint32_t present;
  } Servo;

  std::map<uint8_t,Servo> servos_;
  DYNAMIXEL::ControlTableItemInfo_t ctrlPresentPos_, ctrlGoalPos_, ctrlGoalVel_;
  static const uint16_t userPktBufCap = 128;
  uint8_t userPktBuf[userPktBufCap];

  typedef struct {
    int32_t presentPos;
  } __attribute((packed)) srData_t;
  typedef struct {
    int32_t goalPos;
  } __attribute((packed)) swData_t;

  DYNAMIXEL::InfoSyncReadInst_t srInfos;
  DYNAMIXEL::XELInfoSyncRead_t *infoXelsSr;

  DYNAMIXEL::InfoSyncWriteInst_t swInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSw;

  void setupSyncBuffers();
  void teardownSyncBuffers();
};

#endif // BB8SERVOS_H