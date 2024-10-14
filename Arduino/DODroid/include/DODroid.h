#if !defined(DODROID_H)
#define DODROID_H

#include <LibBB.h>
#include "DODriveController.h"

using namespace bb;

class DODroid: public Subsystem, public PacketReceiver {
public:
  static DODroid droid;

  struct Params {
    float wheelSpeedKp, wheelSpeedKi, wheelSpeedKd;
    float balKp, balKi, balKd;
    float pwmBalKp, pwmBalKi, pwmBalKd;
    float accel, pwmAccel;
    float maxSpeed;
    float faNeckIMUAccel, faNeckRemoteAccel, faNeckSpeed;
    float faHeadRollTurn, faHeadHeadingTurn;
    float faAntennaSpeed;
    float annealHeadTime;
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

  DODroid();
  virtual Result initialize();
  virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  Result stepPowerProtect();
  Result stepDrive();
  Result stepHead();

  virtual void printStatus(ConsoleStream *stream);
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

protected:
  bb::IMU imu_;
  bb::DCMotor leftMotor_, rightMotor_;
  bb::Encoder leftEncoder_, rightEncoder_;
  bb::PIDController lSpeedController_, rSpeedController_;

  bb::IMUControlInput balanceInput_;
  DODriveControlOutput driveOutput_, pwmDriveOutput_;
  bb::PIDController balanceController_, pwmBalanceController_;
  
  MotorStatus leftMotorStatus_, rightMotorStatus_;
  
  bool pwm_;
  bool driveOn_;
  bool servosOK_, antennasOK_;
  bool lastBtn0_, lastBtn1_, lastBtn2_, lastBtn3_, lastBtn4_;
  float remoteP_, remoteH_, remoteR_;
  float remoteP0_, remoteH0_, remoteR0_;
  float annealP_, annealH_, annealR_;
};

#endif