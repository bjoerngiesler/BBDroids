#if !defined(REMOTEDISPLAY_H)
#define REMOTEDISPLAY_H

#if defined(LEFT_REMOTE)

#include <LibBB.h>
#include <SoftwareSerial.h>

#include "RemoteInput.h"

using namespace bb;

class RemoteDisplay: public Subsystem, public RemoteInput::Delegate {
public:
  typedef enum {
    LOGO_SCREEN,
    LEFT_REMOTE_SCREEN,
    RIGHT_REMOTE_SCREEN,
    DROID_STATUS_SCREEN,
    MENU_SCREEN
  } Screen;

  static RemoteDisplay display;

  virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();

  Screen currentScreen() { return curScreen_; }
  Result showScreen(Screen screen);

  Result update();
  void setConnected(bool conn);

  void setLastPacketFromDroid(const Packet& packet) { lastPacketFromDroid_ = packet; }
  void setLastPacketFromRightRemote(const Packet& packet) { lastPacketFromRightRemote_ = packet; }

  virtual void buttonTopLeftPressed();
  virtual void buttonTopRightPressed();

protected:
  RemoteDisplay();
  virtual ~RemoteDisplay() {}
  String sendStringAndWaitForResponse(const String& str, int predelay=0, bool nl=true);
  bool sendStringAndWaitForOK(const String& str, int predelay=0, bool nl=true);
  bool readString(String& str, unsigned char terminator='\n');
  Result drawCursor(int x, int y, int width, uint16_t color);

  SerialPIO ser_;
  Screen curScreen_;
  bool connected_;
  bool left_led_state_, right_led_state_;
  unsigned long last_millis_;
  int topX_, topY_, bottomX_, bottomY_;
  float headingX_, headingY_;
  Packet lastPacketFromDroid_, lastPacketFromRightRemote_;
};

#endif // LEFT_REMOTE
#endif // REMOTEDISPLAY_H