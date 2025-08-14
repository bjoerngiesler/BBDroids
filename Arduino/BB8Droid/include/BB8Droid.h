#if !defined(BB8_H)
#define BB8_H

#include <LibBB.h>
#include <Adafruit_NeoPixel.h>

#include "BB8Config.h"

using namespace bb;

class BB8: public Subsystem, public PacketReceiver {
public:
  static BB8 bb8;
  static BB8Params params_;

  typedef enum {
    MODE_OFF,
    MODE_ROLL_CONTROL_ONLY,
    MODE_SPEED_CONTROL_ONLY,
    MODE_SPEED_ROLL_CONTROL,
    MODE_POS_CONTROL,
    MODE_KIOSK,
    MODE_CALIB
  } Mode;

  BB8();
  virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result stepIfNotStarted();
  virtual Result stepDriveMotor();
  virtual Result stepRollMotor();
  virtual Result stepDome();
  virtual Result setMode(Mode mode);

  Result selfTest(ConsoleStream *stream = NULL);
  Result servoTest(ConsoleStream *stream = NULL);

  virtual void printStatus(ConsoleStream *stream);

  //virtual Result incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet);
  virtual Result incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet);

  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result fillAndSendStatePacket();

  virtual Result setParameterValue(const String& name, const String& stringVal);

  void printCurrentSystemStatus(ConsoleStream *stream = NULL);


  enum WhichLED {
    LED_STATUS = 0,
    LED_COMM   = 1,
    LED_DRIVE  = 2
  };
  enum WhatColor {
    OFF    = 0,
    RED    = 1,
    GREEN  = 2,
    BLUE   = 3,
    YELLOW = 4,
    WHITE  = 5
  };
  Result setLED(WhichLED which, uint8_t r, uint8_t g, uint8_t b);
  Result setLED(WhichLED which, WhatColor color);
  void setLEDBrightness(uint8_t brightness);


protected:
  void setControlParameters();
  void setServoParameters();

  float driveSpeedGoal_, bodyRollGoal_, domePitchGoal_, domeRollGoal_;
  bool pwmControl_;

  ConfigStorage::HANDLE paramsHandle_;
  bb::ControlPacket lastPacket_;
  int packetTimeout_;

  int numLeftCtrlPackets_, numRightCtrlPackets_;
  uint8_t lastLeftSeqnum_, lastRightSeqnum_;
  unsigned long msLastLeftCtrlPacket_, msLastRightCtrlPacket_, msLastPrimaryCtrlPacket_;
  bb::ControlPacket lastPrimaryCtrlPacket_, lastSecondaryCtrlPacket_;


  size_t packetsReceived_, packetsMissed_;
  bool runningStatus_;

  typedef enum {
    DOME_SERVO_NONE,
    DOME_SERVO_PITCH,
    DOME_SERVO_ROLL,
    DOME_SERVO_BOTH
  } DomeServoType;

  bb::IMU imu_;
  
  DomeServoType servoDomeToIMU_;
  unsigned int kioskDelay_;

  Mode mode_;

  bb::DCMotor driveMotor_, yawMotor_;
  bb::Encoder driveEncoder_;
  bb::IMUControlInput balanceInput_, rollInput_;
  bb::ServoControlOutput rollOutput_;
  bb::PIDController driveController_, balanceController_, rollController_;

  float remoteP_, remoteH_, remoteR_;
  float annealP_, annealH_, annealR_, annealTime_;

  Adafruit_NeoPixel statusPixels_;
  bool commLEDOn_;


  bool servosOK_;
};

#endif // BB8_H