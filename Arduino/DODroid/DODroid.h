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
    float driveAccel;
  };
  static Params params_;

  DODroid();
  virtual Result initialize();
  virtual Result start(ConsoleStream *stream = NULL);
  virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();

  virtual void printStatus(ConsoleStream *stream);
  virtual Result fillAndSendStatePacket();

  virtual Result incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet);
  virtual Result incomingConfigPacket(uint16_t station, PacketSource source, uint8_t rssi, const ConfigPacket& packet);
  virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  virtual Result setParameterValue(const String& name, const String& stringVal);

  Result selfTest();

protected:
  bb::DCMotor leftMotor_, rightMotor_;
  bb::Encoder leftEncoder_, rightEncoder_;
  
  bb::PIDController* balanceController_;
  DOIMUControlInput* balanceInput_;
  DODriveControlOutput* driveOutput_;
  
  bool motorsOK_;
};

#endif