#if !defined(DODROID_H)
#define DODROID_H

#include <LibBB.h>
#include <Adafruit_NeoPixel.h>

#include "DODriveController.h"
#include "DOConfig.h"

using namespace bb;

class DODroid: public Subsystem, public PacketReceiver {
public:
  static DODroid droid;
  static DOParams params_;
  static bb::ConfigStorage::HANDLE paramsHandle_;

  enum MotorStatus { // FIXME this probably needs to go somewhere else, closer to DCMotor or EncoderMotor?
    MOTOR_UNTESTED         = 0,
    MOTOR_OK               = 1,
    MOTOR_DISCONNECTED     = 2,
    MOTOR_ENC_DISCONNECTED = 3,
    MOTOR_REVERSED         = 4,
    MOTOR_ENC_REVERSED     = 5,
    MOTOR_BOTH_REVERSED    = 6,
    MOTOR_BLOCKED          = 7,
    MOTOR_OTHER            = 8 // for example IMU broken
  };

  const char* motorStatusToString(MotorStatus status) const {
    switch(status) {
    case MOTOR_UNTESTED:          return "untested"; break;
    case MOTOR_OK:                return "OK"; break;
    case MOTOR_DISCONNECTED:      return "disconnected"; break;
    case MOTOR_ENC_DISCONNECTED:  return "encoder disconnected"; break;
    case MOTOR_REVERSED:          return "reversed"; break;
    case MOTOR_ENC_REVERSED:      return "encoder reversed"; break;
    case MOTOR_BOTH_REVERSED:     return "motor and encoder reversed"; break;
    case MOTOR_BLOCKED:           return "blocked"; break;
    case MOTOR_OTHER: default:    return "unknown error"; break;
    }
    return "???";
  }

  DODroid();
  virtual Result initialize();
  virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result stepIfNotStarted();
  Result stepPowerProtect();
  Result stepDrive();
  Result stepHead();

  virtual String statusLine();
  virtual void printExtendedStatus(ConsoleStream *stream = NULL);
  virtual Result fillAndSendStatePacket();

  void setControlParameters();

  enum DriveMode {
    DRIVE_OFF      = 0,
    DRIVE_VEL      = 1,
    DRIVE_POS      = 2,
    DRIVE_AUTO_POS = 3
  };

  void switchDrive(DriveMode mode);

  virtual Result incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet);
  virtual Result incomingConfigPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, ConfigPacket& packet);
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result setParameterValue(const String& name, const String& stringVal);

  Result selfTest(ConsoleStream *stream = NULL);
  Result servoTest(ConsoleStream *stream = NULL);
  Result motorTest(ConsoleStream *stream = NULL);
  MotorStatus singleMotorTest(bb::DCMotor& mot, bb::Encoder& enc, bool reverse, ConsoleStream *stream = NULL);

  bool aerialsOK() { return aerialsOK_; }
  bool setAerials(uint8_t a1, uint8_t a2, uint8_t a3, bool update=false);
  bool getAerials(uint8_t& a1, uint8_t& a2, uint8_t& a3);

  bool setEyes(uint8_t lCol, uint8_t lSize, uint8_t lPos, uint8_t rCol, uint8_t rSize, uint8_t rPos, bool update=false);
  bool setControlStrip(uint8_t strip, bool update=false);

  bool updateHead();

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
  bb::IMU imu_;
  bb::DCMotor leftMotor_, rightMotor_;
  bb::Encoder leftEncoder_, rightEncoder_;
  bb::PIDController lSpeedController_, rSpeedController_;

  bb::IMUControlInput balanceInput_;
  DOVelControlOutput velOutput_;
  bb::PIDController balanceController_;
  DOPosControlInput posInput_;
  DOPosControlOutput posOutput_;
  bb::PIDController autoPosController_, posController_; 
  float posControllerZero_;
  
  MotorStatus leftMotorStatus_, rightMotorStatus_;

  bb::LowPassFilter leanFilter_;

  int numLeftCtrlPackets_, numRightCtrlPackets_;
  uint8_t lastLeftSeqnum_, lastRightSeqnum_;
  unsigned long msLastLeftCtrlPacket_, msLastRightCtrlPacket_, msLastPrimaryCtrlPacket_;
  bb::ControlPacket lastPrimaryCtrlPacket_, lastSecondaryCtrlPacket_;
  float msSinceDriveInput_;

  DriveMode driveMode_;
  bool driveSafety_;
  
  bool servosOK_, aerialsOK_;
  float remoteP_, remoteH_, remoteR_;
  float pitchAtRest_;
  float remoteAerial1_, remoteAerial2_, remoteAerial3_;
  float annealP_, annealH_, annealR_, annealTime_;
  float lean_;
  bool headIsOn_;

  Adafruit_NeoPixel statusPixels_;
  bool commLEDOn_;
};

#endif