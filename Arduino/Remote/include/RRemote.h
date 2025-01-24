#if !defined(RREMOTE_H)
#define RREMOTE_H

#include <LibBB.h>
#include <deque>
#include "Config.h"

#include "RWidget.h"
#include "RMenuWidget.h"
#include "RMessageWidget.h"
#include "RGraphsWidget.h"
#include "RCrosshairWidget.h"
#include "RLabelWidget.h"
#include "RIMUWidget.h"
#include "RRemoteVisWidget.h"
#include "RRotaWidget.h"
#include "RSeqnumWidget.h"

using namespace bb;

class RRemote: public Subsystem, public PacketReceiver {
public:
  static RRemote remote;

  enum Mode {
    MODE_REGULAR       = 0,
    MODE_CONTROLSILENT = 1,
    MODE_CALIBRATION   = 2
  };

  Result initialize();
  Result start(ConsoleStream *stream = NULL);
  Result stop(ConsoleStream *stream = NULL);
  Result step();
  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
	virtual void parameterChangedCallback(const String& name);

  Result incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet);
  Result incomingStatePacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const StatePacket& packet);
  Result incomingConfigPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ConfigPacket& packet);
  Result fillAndSend();
  
  void updateStatusLED();
  virtual String statusLine();
  virtual void printExtendedStatus(ConsoleStream *stream = NULL);
  void printExtendedStatusLine(ConsoleStream *stream = NULL); 


  void setMainWidget(RWidget* widget);

  void showPairDroidMenu();
  void showPairRemoteMenu();
  void showMenu(RMenuWidget* menu);
  void showGraphs();
  void showMain();

  void populateMenus();
  void drawGUI();

  void setTopTitle(const String& title);

  // Pairing callbacks
  void selectDroid(const HWAddress& address);
  void selectRightRemote(const HWAddress& address);

  // Other callbacks
  void setIncrRotButtonCB(RInput::Button button, bool left);
  void factoryResetCB(bool left);
  void startCalibrationCB(bool left);
  void finishCalibrationCB(bool left);
  void setLeftIsPrimaryCB(bool yesno);

  void factoryReset();
  void startCalibration();
  void finishCalibration();

  void storeParams();

  void showMessage(const String& str, unsigned int delayms=0, uint8_t color=RDisplay::WHITE);

#if defined(LEFT_REMOTE)
  Result sendConfigToRightRemote();
#endif

  void runTestsuite();

protected:
  RRemote();

  static const unsigned int MSGDELAY = 2000;

  Mode mode_;
  
  bool runningStatus_;
  Packet lastPacketSent_;

  RMenuWidget mainMenu_, pairMenu_, pairDroidMenu_, pairRemoteMenu_, leftRemoteMenu_, rightRemoteMenu_, droidMenu_;
  RMenuWidget lRIncrRotMenu_, rRIncrRotMenu_;
  RMessageWidget message_;
  RLabelWidget topLabel_, bottomLabel_;
  RSeqnumWidget leftSeqnum_, rightSeqnum_, droidSeqnum_;
  RRotaWidget mainVis_;
  RRemoteVisWidget remoteVisL_, remoteVisR_;

  RWidget* mainWidget_;

  bool needsMenuRebuild_;
  std::vector<XBee::Node> discoveredNodes_;

  RWidget titleWidget;

#if defined(LEFT_REMOTE)
  Packet lastPacketFromDroid_, lastPacketFromRightRemote_;
#endif

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
  
  uint8_t lastRightSeqnum_, lastDroidSeqnum_;
  unsigned long lastRightMs_, lastDroidMs_;
};

#endif // RREMOTE_H