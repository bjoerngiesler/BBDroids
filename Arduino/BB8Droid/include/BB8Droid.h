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

  virtual void printStatus(ConsoleStream *stream);

  virtual Result incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet);

  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result fillAndSendStatusPacket();

  virtual void parameterChangedCallback(const String& name);

  void printCurrentSystemStatus(ConsoleStream *stream = NULL);

protected:
  typedef struct {
    float driveSpeedKp, driveSpeedKi, driveSpeedKd;
    float balKp, balKi, balKd;
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
  bb::PIDController driveController_;
};

#endif // BB8_H