#if !defined(BB8_H)
#define BB8_H

#include <LibBB.h>

using namespace bb;

class BB8: public Subsystem, public PacketReceiver {
public:
  static BB8 bb8;

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
  virtual Result stepDriveMotor();
  virtual Result stepRollMotor();
  virtual Result stepDome();
  virtual Result setMode(Mode mode);

  Result selfTest(ConsoleStream *stream = NULL);
  Result servoTest(ConsoleStream *stream = NULL);

  virtual void printStatus(ConsoleStream *stream);

  virtual Result incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet);

  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result fillAndSendStatePacket();

  virtual Result setParameterValue(const String& name, const String& stringVal);

  void printCurrentSystemStatus(ConsoleStream *stream = NULL);

protected:
  void setControlParameters();

  typedef struct {
    float driveSpeedKp, driveSpeedKi, driveSpeedKd;
    float balKp, balKi, balKd;
    float rollKp, rollKi, rollKd;
    float rollServoKp, rollServoKi, rollServoKd;
    float rollServoVel;
  } BB8Params;

  static BB8Params params_;

  float driveSpeedGoal_, bodyRollGoal_, domePitchGoal_, domeRollGoal_;
  bool pwmControl_;

  ConfigStorage::HANDLE paramsHandle_;
  bb::ControlPacket lastPacket_;
  int packetTimeout_;
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
  bb::ServoControlOutput rollOutput_;
  bb::IMUControlInput balanceInput_, rollInput_;
  bb::PIDController driveController_, balanceController_, rollController_;

  bool servosOK_;
};

#endif // BB8_H