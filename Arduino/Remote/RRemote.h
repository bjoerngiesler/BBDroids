#if !defined(RREMOTE_H)
#define RREMOTE_H

#include <Adafruit_NeoPixel.h>
#include <LibBB.h>
#include "Config.h"
#include "RMenu.h"
#include "RMessage.h"
#include "RGraphs.h"
#include "IMUFilter.h"

using namespace bb;

class RRemote: public Subsystem, public PacketReceiver {
public:
  static RRemote remote;

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
  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  Result incomingPacket(uint16_t source, uint8_t rssi, const Packet& packet);
  Result fillAndSend();
  void printStatus(ConsoleStream *stream = NULL);

  void showMainMenu() { showMenu(mainMenu_); }
  void showSettingsMenu() { showMenu(settingsMenu_); }
  void showDroidsMenu();
  void showRemotesMenu();
  void showMenu(RMenu* menu);
  void showGraphs();

  void selectDroid(uint16_t stationId);
  void selectRightRemote(uint16_t stationId);

protected:
  RRemote();

  bool runningStatus_;
  Adafruit_NeoPixel statusPixels_;
  bool onInitScreen_;
  Packet lastPacketSent_;
  RMenu *mainMenu_, *settingsMenu_, *droidsMenu_, *remotesMenu_;
  RGraphs *graphs_;
  RMessage *waitMessage_;
  RDrawable *currentDrawable_;
  bool needsDraw_;
  std::vector<XBee::Node> discoveredNodes_;

#if defined(LEFT_REMOTE)
  Packet lastPacketFromDroid_, lastPacketFromRightRemote_;
#endif

  float deltaR_, deltaP_, deltaH_;

  struct RemoteParams {
    uint16_t leftID;
    uint16_t rightID;
    uint16_t droidID;
  };

  static RemoteParams params_;
};

#endif // RREMOTE_H