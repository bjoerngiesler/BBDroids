#if !defined(BB8_H)
#define BB8_H

#include <LibBB.h>

using namespace bb;

class BB8: public Subsystem, public PacketReceiver {
public:
  static BB8 bb8;

  BB8();
  virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
  virtual Result incomingPacket(const Packet& packet);
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	virtual Result parameterValue(const String& name, String& value);
	virtual Result setParameterValue(const String& name, const String& value);

  void printStatus(ConsoleStream *stream = NULL);

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
  ConfigStorage::HANDLE paramsHandle_;
  Packet lastPacket_;
  int packetTimeout_;
  size_t packetsReceived_, packetsMissed_;
  bool runningStatus_;
  bool kioskMode_;
  unsigned int kioskDelay_;
};

#endif // BB8_H