#if !defined(DOSERVOS_H)
#define DOSERVOS_H

#include <LibBB.h>
#include <DynamixelShield.h>
#include <vector>

using namespace bb;

class DOServoControlOutput: public bb::ControlOutput {
public:
  DOServoControlOutput(uint8_t servoNum, float offset=0.0f);
  float present();
  bb::Result set(float value);

protected:
  uint8_t sn_;
  float offset_;
};

class DOServos: public Subsystem {
public:
  typedef enum {
    VALUE_DEGREE,
    VALUE_RAW
  } ValueType;

  static DOServos servos;

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

  bool hasServoWithID(uint8_t id);
  bool setRange(uint8_t id, float min, float max, ValueType t=VALUE_DEGREE);
  bool setOffset(uint8_t id, float offset, ValueType t=VALUE_DEGREE);
  bool setInverted(uint8_t id, bool inverted);
  bool inverted(uint8_t id);
  
  bool setGoal(uint8_t id, float goal, ValueType t=VALUE_DEGREE);
  bool setProfileVelocity(uint8_t id, float vel, ValueType t=VALUE_DEGREE); // in this case deg/s, while raw value is in rev/min
  bool setProfileAcceleration(uint8_t id, uint32_t val);
  float goal(uint8_t id, ValueType t=VALUE_DEGREE);
  float present(uint8_t id, ValueType t=VALUE_DEGREE);
  float load(uint8_t id);
  uint8_t errorStatus(uint8_t id);
  bool loadShutdownEnabled(uint8_t id);
  void setLoadShutdownEnabled(uint8_t id, bool yesno);
  
  Result switchTorque(uint8_t id, bool onoff);
  bool isTorqueOn(uint8_t id);

  uint32_t computeRawValue(float val, ValueType t=VALUE_DEGREE);

protected:
  DOServos();
  DynamixelShield dxl_;

  struct Servo {
    uint8_t id;
    uint32_t goal;
    uint32_t profileVel;
    uint32_t present;
    int16_t load;
    uint32_t min, max;
    int16_t offset;
    uint32_t lastVel;
  };

  //std::map<uint8_t,Servo> servos_;
  std::vector<Servo> servos_;
  Servo *servoWithID(uint8_t id);

  std::vector<uint8_t> requiredIds_;
  DYNAMIXEL::ControlTableItemInfo_t ctrlPresentPos_, ctrlGoalPos_, ctrlProfileVel_, ctrlPresentLoad_;
  static const uint16_t userPktBufCap = 128;
  uint8_t userPktBufPresent[userPktBufCap], userPktBufLoad[userPktBufCap];

  typedef struct {
    int32_t presentPos;
  } __attribute((packed)) srDataPresent_t;
  typedef struct {
    int16_t load;
  } __attribute((packed)) srDataLoad_t;
  typedef struct {
    int32_t goalPos;
  } __attribute((packed)) swDataGoal_t;
  typedef struct {
    int32_t profileVel;
  } __attribute((packed)) swDataVel_t;

  DYNAMIXEL::InfoSyncReadInst_t srPresentInfos;
  DYNAMIXEL::XELInfoSyncRead_t *infoXelsSrPresent;

  DYNAMIXEL::InfoSyncReadInst_t srLoadInfos;
  DYNAMIXEL::XELInfoSyncRead_t *infoXelsSrLoad;

  DYNAMIXEL::InfoSyncWriteInst_t swGoalInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSwGoal;

  DYNAMIXEL::InfoSyncWriteInst_t swVelInfos;
  DYNAMIXEL::XELInfoSyncWrite_t *infoXelsSwVel;

  void setupSyncBuffers();
  void teardownSyncBuffers();
  Result syncReadInfo(ConsoleStream *stream = NULL);
  Result syncWriteInfo(ConsoleStream *stream = NULL);
};

#endif // DOSERVOS_H