#if !defined(BB8_H)
#define BB8_H

#include <LibBB.h>
#include "BB8Controllers.h"

using namespace bb;

class BB8: public Subsystem, public PacketReceiver {
public:
  static BB8 bb8;

  typedef enum {
    VAL_TIMESTAMP           =  0,
    VAL_DRIVE_GOAL          =  1,
    VAL_DRIVE_CURRENT_PWM   =  2,
    VAL_DRIVE_CURRENT_SPEED =  3,
    VAL_DRIVE_CURRENT_POS   =  4,
    VAL_DRIVE_ERR           =  5,
    VAL_DRIVE_ERR_I         =  6,
    VAL_DRIVE_ERR_D         =  7,
    VAL_DRIVE_CONTROL       =  8,
    VAL_IMU_RAW_R           =  9,
    VAL_IMU_RAW_P           = 10,
    VAL_IMU_RAW_H           = 11,
    VAL_IMU_FILTERED_R      = 12,
    VAL_IMU_FILTERED_P      = 13,
    VAL_IMU_FILTERED_H      = 14,
    VAL_REMOTE_L_AXIS0      = 15,
    VAL_REMOTE_L_AXIS1      = 16,
    VAL_REMOTE_L_AXIS2      = 17,
    VAL_REMOTE_L_AXIS3      = 18,
    VAL_REMOTE_L_AXIS4      = 19,
    VAL_REMOTE_R_AXIS0      = 20,
    VAL_REMOTE_R_AXIS1      = 21,
    VAL_REMOTE_R_AXIS2      = 22,
    VAL_REMOTE_R_AXIS3      = 23,
    VAL_REMOTE_R_AXIS4      = 24,
    VAL_ROLL_GOAL           = 25,
    VAL_ROLL_CURRENT        = 26,
    VAL_ROLL_ERR            = 27,
    VAL_ROLL_ERR_I          = 28,
    VAL_ROLL_ERR_D          = 29,
    VAL_ROLL_CONTROL        = 30,
    VAL_LAST                = 31
  } ValueIndex;

  typedef struct {
    float val[VAL_LAST];
  } UDPStatusPacket;

  BB8();
  virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result incomingPacket(const Packet& packet);
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result fillAndSendStatusPacket();

	virtual Result parameterValue(const String& name, String& value);
	virtual Result setParameterValue(const String& name, const String& value);

  void printCurrentSystemStatus(ConsoleStream *stream = NULL);

protected:
  typedef struct {
    float drive_speed_factor, turn_speed_factor;
    float body_roll_servo_min, body_roll_servo_max, body_roll_servo_offset, body_roll_servo_speed;
    bool body_roll_servo_invert;
    float dome_pitch_servo_min, dome_pitch_servo_max, dome_pitch_servo_offset, dome_pitch_servo_speed;
    bool dome_pitch_servo_invert;
    float dome_roll_servo_min, dome_roll_servo_max, dome_roll_servo_offset, dome_roll_servo_speed;
    bool dome_roll_servo_invert;
    float dome_heading_servo_min, dome_heading_servo_max, dome_heading_servo_offset, dome_heading_servo_speed;
    bool dome_heading_servo_invert;
  } BB8Params;

  BB8Params params_;

  float domeRollKp_, domeRollKi_, domeRollKd_;
  float domePitchKp_, domePitchKi_, domePitchKd_;

  ConfigStorage::HANDLE paramsHandle_;
  Packet lastPacket_;
  int packetTimeout_;
  size_t packetsReceived_, packetsMissed_;
  bool runningStatus_;
  bool kioskMode_;

  typedef enum {
    DOME_SERVO_NONE,
    DOME_SERVO_PITCH,
    DOME_SERVO_ROLL,
    DOME_SERVO_BOTH
  } DomeServoType;

  DomeServoType servoDomeToIMU_;
  unsigned int kioskDelay_;

  bool rollControlOn_;

  BB8IMUControlInput rollControlInput_;
  BB8ServoControlOutput rollControlOutput_;
  BB8PIDController rollController_;
};

#endif // BB8_H