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

  DODroid();
  virtual Result initialize();
  virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  Result stepPowerProtect();
  Result stepDrive();
  Result stepHead();

  virtual String statusLine();
  virtual void printExtendedStatus(ConsoleStream *stream = NULL);
  virtual Result fillAndSendStatePacket();

  void setControlParameters();
  void switchDrive(bool onoff);

  virtual Result incomingControlPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const ControlPacket& packet);
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result setParameterValue(const String& name, const String& stringVal);

  Result selfTest(ConsoleStream *stream = NULL);
  Result servoTest(ConsoleStream *stream = NULL);
  Result motorTest(ConsoleStream *stream = NULL);
  MotorStatus singleMotorTest(bb::DCMotor& mot, bb::Encoder& enc, bool reverse, ConsoleStream *stream = NULL);

  bool antennasOK() { return antennasOK_; }
  bool setAntennas(uint8_t a1, uint8_t a2, uint8_t a3);
  bool getAntennas(uint8_t& a1, uint8_t& a2, uint8_t& a3);

  enum WhichLED {
    LED_STATUS = 0,
    LED_COMM   = 1,
    LED_DRIVE  = 2
  };
  enum WhatColor {
    OFF   = 0,
    RED   = 1,
    GREEN = 2,
    BLUE  = 3,
    WHITE = 4 
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
  DODriveControlOutput driveOutput_;
  bb::PIDController balanceController_;
  
  MotorStatus leftMotorStatus_, rightMotorStatus_;

  int numLeftCtrlPackets_, numRightCtrlPackets_;
  unsigned long msLastLeftCtrlPacket_, msLastRightCtrlPacket_;

  bool driveOn_;
  bool servosOK_, antennasOK_;
  bool lastBtn0_, lastBtn1_, lastBtn2_, lastBtn3_, lastBtn4_;
  float remoteP_, remoteH_, remoteR_;
  float annealP_, annealH_, annealR_, annealTime_;
  float lean_;

  Adafruit_NeoPixel statusPixels_;
  bool commLEDOn_;
};

#endif