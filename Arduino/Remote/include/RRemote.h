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

using namespace bb;

class RRemote: public Subsystem, public PacketReceiver {
public:
  static RRemote remote;

  enum Mode {
    MODE_REGULAR       = 0,
    MODE_CONTROLSILENT = 1,
    MODE_CALIBRATION   = 2
  };

  uint16_t stationID() {
#if defined(LEFT_REMOTE)
    return params_.leftID;
#else
    return params_.rightID;
#endif
  }

  Result initialize();
  Result start(ConsoleStream *stream = NULL);
  Result stop(ConsoleStream *stream = NULL);
  Result step();
  Result stepCalib();
  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  Result incomingPacket(uint16_t source, uint8_t rssi, const Packet& packet);
  Result fillAndSend();
  
  void updateStatusLED();
  void printStatus(ConsoleStream *stream = NULL);

  void setMainWidget(RWidget* widget);

  void showPairDroidMenu();
  void showPairRemoteMenu();
  void showMenu(RMenuWidget* menu);
  void showGraphs();
  void showMain();

  void populateMenus();

  void setTopTitle(const String& title);
  void setBottomTitle(const String& title);

  void selectDroid(uint16_t stationId);
  void selectRightRemote(uint16_t stationId);

  void factoryReset(bool thisremote);
  void startCalibration(bool thisremote);
  void finishCalibration(bool thisremote);

protected:
  RRemote();

  Mode mode_;
  
  bool runningStatus_;
  bool onInitScreen_;
  Packet lastPacketSent_;

  RMenuWidget mainMenu_, pairMenu_, pairDroidMenu_, pairRemoteMenu_, leftRemoteMenu_, rightRemoteMenu_, droidMenu_;
  RMessageWidget waitMessage_, restartMessage_;
  RLabelWidget topLabel_, bottomLabel_;
  RRotaWidget mainVis_;
  RRemoteVisWidget remoteVisL_, remoteVisR_;

  RWidget* mainWidget_;

  bool needsDraw_;
  std::vector<XBee::Node> discoveredNodes_;
  bb::IMU imu_;

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
    uint16_t leftID;
    uint16_t rightID;
    uint16_t droidID;
    RInput::AxisCalib hCalib, vCalib;
  };

  static RemoteParams params_;
  static bb::ConfigStorage::HANDLE paramsHandle_;
};

#endif // RREMOTE_H