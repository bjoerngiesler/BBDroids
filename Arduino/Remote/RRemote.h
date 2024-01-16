#if !defined(RREMOTE_H)
#define RREMOTE_H

#include <Adafruit_NeoPixel.h>
#include <LibBB.h>
#include "Config.h"
#include "RMenu.h"
#include "RMessage.h"
#include "IMUFilter.h"

using namespace bb;

class RRemote: public Subsystem, public PacketReceiver {
public:
  static RRemote remote;

  Result initialize();
  Result start(ConsoleStream *stream = NULL);
  Result stop(ConsoleStream *stream = NULL);
  Result step();
  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
  Result incomingPacket(const Packet& packet);
  void printStatus(ConsoleStream *stream = NULL);

  void showMainMenu() { showMenu(mainMenu_); }
  void showSettingsMenu() { showMenu(settingsMenu_); }
  void showStatus() {}
  void showDroidsMenu();
  void showRemotesMenu();
  void showMenu(RMenu* menu);

  void selectDroid(uint16_t stationId);
  void selectRightRemote(uint16_t stationId);

protected:
  RRemote();

  bool runningStatus_;
  Adafruit_NeoPixel statusPixels_;
  bool onInitScreen_;
  Packet lastPacketSent_;
  RMenu *mainMenu_, *settingsMenu_, *droidsMenu_, *remotesMenu_;
  RMessage *waitMessage_;
  RDrawable *currentDrawable_;
  bool needsDraw_;
  std::vector<XBee::Node> discoveredNodes_;

#if defined(LEFT_REMOTE)
  Packet lastPacketFromDroid_, lastPacketFromRightRemote_;
#endif

  float deltaR_, deltaP_, deltaH_;

};

#endif // RREMOTE_H