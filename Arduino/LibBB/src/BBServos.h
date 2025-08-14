#if !defined(BBSERVOS_H)
#define BBSERVOS_H

#include "BBControllers.h"
#include "BBConsole.h"
#include <DynamixelShield.h>
#include <vector>

namespace bb {

class Servos: public Subsystem {
public:
  typedef enum {
    VALUE_DEGREE,
    VALUE_RAW,
  } ValueType;
  static const ValueType VALUE_COOKED = VALUE_DEGREE;

  enum ControlMode {
    CONTROL_POSITION = 0,
    CONTROL_VELOCITY = 1,
    CONTROL_CURRENT  = 2,
    CONTROL_UNKNOWN  = 255
  };

  static Servos servos;

  static const uint8_t ID_ALL = 255;

	virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  Result handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String>& words, ConsoleStream *stream);

  void setRequiredIds(const std::vector<uint8_t>& ids) { requiredIds_ = ids; }

  Result home(uint8_t id, float vel, unsigned int maxLoadPercent, ConsoleStream* stream = NULL);

  void printStatus(ConsoleStream* stream, int id);

  void setTorqueOffOnStop(bool yesno) { torqueOffOnStop_ = yesno; }
  bool torqueOffOnStop() { return torqueOffOnStop_; }

  bool hasServoWithID(uint8_t id);
  bool setRange(uint8_t id, float min, float max, ValueType t=VALUE_DEGREE);
  bool setOffset(uint8_t id, float offset, ValueType t=VALUE_DEGREE);
  bool setInverted(uint8_t id, bool inverted);
  bool inverted(uint8_t id);
  
  bool setControlMode(uint8_t id, ControlMode mode);
  ControlMode controlMode(uint8_t id);

  bool setGoalPos(uint8_t id, float goal, ValueType t=VALUE_DEGREE);
  bool setProfileVelocity(uint8_t id, float vel, ValueType t=VALUE_DEGREE); // in this case deg/s, while raw value is in rev/min
  bool setProfileAcceleration(uint8_t id, uint32_t val);
  float goalPos(uint8_t id, ValueType t=VALUE_DEGREE);
  float presentPos(uint8_t id, ValueType t=VALUE_DEGREE);

  bool setGoalVel(uint8_t id, float vel, ValueType t=VALUE_DEGREE);
  float goalVel(uint8_t id, ValueType t=VALUE_DEGREE);
  bool setGoalCur(uint8_t id, float cur, ValueType t=VALUE_COOKED);
  float goalCur(uint8_t id, ValueType t=VALUE_COOKED);

  float load(uint8_t id);
  uint8_t errorStatus(uint8_t id);
  bool loadShutdownEnabled(uint8_t id);
  void setLoadShutdownEnabled(uint8_t id, bool yesno);

  bool setPIDValues(uint8_t id, uint16_t kp, uint16_t ki, uint16_t kd);
  
  Result switchTorque(uint8_t id, bool onoff);
  bool isTorqueOn(uint8_t id);

  uint32_t computeRawValue(float val, ValueType t=VALUE_DEGREE);

  Result write() { return syncWriteInfo(); }

protected:
  Servos();
  DynamixelShield dxl_;

  struct Servo {
    uint8_t id;
    ControlMode mode;
    uint32_t goalPos;
    uint32_t profileVel;
    uint32_t presentPos;
    int32_t goalVel;
    int16_t goalCur;
    int16_t load;
    uint32_t min, max;
    int16_t offset;
    uint32_t lastVel;
  };

  std::vector<Servo> servos_;
  Servo *servoWithID(uint8_t id);

  std::vector<uint8_t> requiredIds_;
  DYNAMIXEL::ControlTableItemInfo_t ctrlPresentPos_, ctrlGoalPos_, ctrlProfileVel_, ctrlPresentLoad_, ctrlGoalVel_, ctrlGoalCur_;
  static const uint16_t userPktBufCap = 128;
  uint8_t userPktBufPresent[userPktBufCap], userPktBufLoad[userPktBufCap];
  bool torqueOffOnStop_;

  bool getControlTableItemInfo(uint16_t model, uint8_t item, DYNAMIXEL::ControlTableItemInfo_t& info);

  typedef struct {
    int32_t presentPos;
  } __attribute((packed)) srDataPresent_t;
  typedef struct {
    int16_t load;
  } __attribute((packed)) srDataLoad_t;
  typedef struct {
    int32_t goalPos;
  } __attribute((packed)) swDataGoalPos_t;
  typedef struct {
    int32_t prfVel;
  } __attribute((packed)) swDataPrfVel_t;

  DYNAMIXEL::InfoSyncReadInst_t srPresentInfos;
  DYNAMIXEL::XELInfoSyncRead_t *infoXelsSrPresent;

  DYNAMIXEL::InfoSyncReadInst_t srLoadInfos;
  DYNAMIXEL::XELInfoSyncRead_t *infoXelsSrLoad;

  DYNAMIXEL::InfoSyncWriteInst_t swGoalPosInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSwGoalPos;

  DYNAMIXEL::InfoSyncWriteInst_t swPrfVelInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSwPrfVel;

  DYNAMIXEL::InfoSyncWriteInst_t swGoalVelInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSwGoalVel;

  DYNAMIXEL::InfoSyncWriteInst_t swGoalCurInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSwGoalCur;

  void setupSyncBuffers();
  void teardownSyncBuffers();
  Result syncReadInfo(ConsoleStream *stream = NULL);
  Result syncWriteInfo(ConsoleStream *stream = NULL);
};

class ServoControlOutput: public bb::ControlOutput {
public:
  ServoControlOutput(uint8_t servoNum, float offset=0.0f, Servos::ControlMode mode=Servos::CONTROL_POSITION);
  float present();
  bb::Result set(float value);

protected:
  uint8_t sn_;
  float offset_;
  Servos::ControlMode mode_;
};

};

#endif // DOSERVOS_H