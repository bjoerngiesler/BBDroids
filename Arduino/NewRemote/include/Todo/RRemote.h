#if !defined(RREMOTE_H)
#define RREMOTE_H

#include <LibBB.h>
#include <deque>
#include "Config.h"
#include "RInput.h"

using namespace bb;

class RRemote: public Subsystem, public PacketReceiver {
public:
  static RRemote remote;

  enum Mode {
    MODE_REGULAR       = 0,
    MODE_CONTROLSILENT = 1,
    MODE_CALIBRATION   = 2
  };

  void setIsLeftRemote();

  Result initialize();
  Result start(ConsoleStream *stream = NULL);
  Result stop(ConsoleStream *stream = NULL);
  Result step();
  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
	virtual void parameterChangedCallback(const String& name);

  Result incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet);
  Result incomingStatePacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const StatePacket& packet);
  Result incomingConfigPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, ConfigPacket& packet);
  Result incomingPairingPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, PairingPacket& packet);
  Result fillAndSend();
  
  void updateStatusLED();
  virtual String statusLine();
  virtual void printExtendedStatus(ConsoleStream *stream = NULL);
  void printExtendedStatusLine(ConsoleStream *stream = NULL); 

  // Pairing callbacks
  void selectDroid(const HWAddress& address);
  void selectRightRemote(const HWAddress& address);

  // Other callbacks
  void setLEDBrightness(uint8_t brt);
  uint8_t ledBrightness() { return params_.config.ledBrightness; }

  void setJoyDeadband(uint8_t db);
  uint8_t joyDeadband() { return params_.config.deadbandPercent; }

  void setSendRepeats(uint8_t sr);
  uint8_t sendRepeats() { return params_.config.sendRepeats; }

  void startCalibration();
  void finishCalibration();
  void factoryReset();
  
  void sendFactoryReset();

  Result sendConfigToRightRemote();
  void storeParams() { ConfigStorage::storage.writeBlock(paramsHandle_);  }
  void runTestsuite();

  const HWAddress& droidAddress() { return params_.droidAddress; }
  const HWAddress& otherRemoteAddress() { return params_.otherRemoteAddress; }

  RInput::Button incrRotButton(PacketSource source);
  void setIncrRotButton(PacketSource source, RInput::Button btn);

  PacketSource primary() { return params_.config.leftIsPrimary ? PACKET_SOURCE_LEFT_REMOTE : PACKET_SOURCE_RIGHT_REMOTE; }
  void setLeftIsPrimary(bool yesno);
  bool isPrimary() { return (isLeftRemote && params_.config.leftIsPrimary) || (!isLeftRemote && !params_.config.leftIsPrimary);}

  // Callbacks
  void setIncrRotButtonCB(RInput::Button button, bool left);
  void sendStartCalibration();
  void finishCalibrationCB(bool left);


protected:
  RRemote();

  static const unsigned int MSGDELAY = 2000;

  Mode mode_;
  
  bool runningStatus_;
  Packet lastPacketSent_;

  Packet lastPacketFromDroid_, lastPacketFromRightRemote_;

  float deltaR_, deltaP_, deltaH_;

  static const uint16_t MAX_CALIB_ROUNDS_BEFORE_ABORT = 1000;
  static const uint8_t MAX_CALIB_BUFSIZE = 100;
  static const uint8_t MAX_CALIB_DIFF = 30;
  std::deque<uint16_t> rawHBuf, rawVBuf;
  RInput::AxisCalib hCalib, vCalib;
  int16_t calibRounds;

  struct RemoteParams {
    HWAddress otherRemoteAddress = {0, 0};
    HWAddress droidAddress = {0, 0};
    RInput::AxisCalib hCalib = RInput::AxisCalib();
    RInput::AxisCalib vCalib = RInput::AxisCalib();
    bb::RemoteConfigPacket config;
  };

  static RemoteParams params_;
  static bb::ConfigStorage::HANDLE paramsHandle_;
  unsigned int ledBrightness_, deadbandPercent_, sendRepeats_;
  
  unsigned long lastRightMs_, lastDroidMs_;
};

#endif // RREMOTE_H