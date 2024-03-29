#if !defined(DODROID_H)
#define DODROID_H

#include <LibBB.h>
#include "DOIMU.h"
#include "DODriveController.h"

using namespace bb;

class DODroid: public Subsystem, public PacketReceiver {
public:
  static DODroid droid;
  
  struct Params {
    float balKp, balKi, balKd;
    float speedKp, speedKi, speedKd;
    float posKp, posKi, posKd;
    float speedRemoteFactor, rotRemoteFactor;
  };
  static Params params_;

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

  enum DriveMode {
    DRIVE_OFF                 = 0,
    DRIVE_NAIVE               = 1,
    DRIVE_PITCH               = 2,
    DRIVE_PITCH_SPEED         = 3,
    DRIVE_PITCH_HEADING_SPEED = 4
  };

  DODroid();
  virtual Result initialize();
  virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();

  Result stepDriveNaive();
  Result stepDrivePitch();
  Result stepDrivePitchSpeed();
  Result stepDrivePitchHeadingSpeed();

  virtual void printStatus(ConsoleStream *stream);
  virtual Result fillAndSendStatePacket();

  virtual Result incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet);
  virtual Result incomingConfigPacket(uint16_t station, PacketSource source, uint8_t rssi, const ConfigPacket& packet);
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result setParameterValue(const String& name, const String& stringVal);

  Result selfTest(ConsoleStream *stream = NULL);
  Result servoTest(ConsoleStream *stream = NULL);
  Result motorTest(ConsoleStream *stream = NULL);
  MotorStatus singleMotorTest(bb::DCMotor& mot, bb::Encoder& enc, bool reverse, ConsoleStream *stream = NULL);

protected:
  bb::DCMotor leftMotor_, rightMotor_;
  bb::Encoder leftEncoder_, rightEncoder_;
  
  bb::PIDController* balanceController_;
  DOIMUControlInput* balanceInput_;
  DODriveControlOutput* driveOutput_;
  
  MotorStatus leftMotorStatus_, rightMotorStatus_;

  DriveMode driveMode_;
  
  bool servosOK_;
};

#endif