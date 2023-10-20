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
    VALUE_DEGREE,
    VALUE_RAW
  } ValueType;

  static BB8Servos servos;

  static const uint8_t ID_ALL = 255;

	virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  Result handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String>& words, ConsoleStream *stream);

  Result runServoTest(ConsoleStream *stream, int id);
  
  void printStatus(ConsoleStream* stream, int id);

  bool setRange(uint8_t id, float min, float max, ValueType t=VALUE_DEGREE);
  bool setOffset(uint8_t id, float offset, ValueType t=VALUE_DEGREE);
  bool setInvert(uint8_t id, bool invert);
  
  bool setGoal(uint8_t id, float goal, ValueType t=VALUE_DEGREE);
  bool setProfileAcceleration(uint8_t id, uint32_t val);
  bool setProfileVelocity(uint8_t id, uint32_t val);
  float goal(uint8_t id, ValueType t=VALUE_DEGREE);
  float present(uint8_t id, ValueType t=VALUE_DEGREE);
  uint8_t errorStatus(uint8_t id);
  Result moveSlow(int id, float goal, ValueType t=VALUE_DEGREE);
  
  Result switchTorque(uint8_t id, bool onoff);
  bool isTorqueOn(uint8_t id);

  uint32_t computeRawValue(float val, ValueType t=VALUE_DEGREE);

protected:
  BB8Servos();
  DynamixelShield dxl_;

  typedef struct {
    uint32_t goal;
    uint32_t present;
    uint32_t min, max;
    int32_t offset;
    bool invert;
  } Servo;

  std::map<uint8_t,Servo> servos_;
  DYNAMIXEL::ControlTableItemInfo_t ctrlPresentPos_, ctrlGoalPos_, ctrlProfileVel_;
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