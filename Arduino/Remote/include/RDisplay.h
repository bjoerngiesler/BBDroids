#if !defined(RDISPLAY_H)
#define RDISPLAY_H

#include <LibBB.h>
#if defined(LEFT_REMOTE)
#include <SoftwareSerial.h>
#endif
#include <vector>

#include "RInput.h"

using namespace bb;

class RDrawable {
public:
  virtual Result draw(ConsoleStream* stream = NULL) = 0;
  void setNeedsCls(bool needs) { needsCls_ = needs; }
protected:
  bool needsCls_;
};

class RDisplay: public Subsystem {
public:
  static const uint16_t BLACK = 0x0000;
  static const uint16_t WHITE = 0xffff;
  static const uint16_t RED   = 0xF800;
  static const uint16_t GREEN = 0x0400;
  static const uint16_t BLUE  = 0x001F;
  static const uint16_t YELLOW = 0xFFE0;

  static const uint8_t CHAR_WIDTH = 6;
  static const uint8_t CHAR_HEIGHT = 10;
  static const uint8_t DISPLAY_WIDTH = 80;
  static const uint8_t DISPLAY_HEIGHT = 160;

  static RDisplay display;

  virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();

  Result cls();
  Result text(uint8_t x, uint8_t y, uint16_t color, const String& text);
  Result hline(uint8_t x, uint8_t y, uint8_t width, uint16_t color);
  Result rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color, bool filled=false);
  Result plot(uint8_t x, uint8_t y, uint16_t color);

protected:
  RDisplay();
  virtual ~RDisplay() {}
  bool readString(String& str, unsigned char terminator='\n');

  String sendStringAndWaitForResponse(const String& str, int predelay=0, bool nl=true);
  bool sendStringAndWaitForOK(const String& str, int predelay=0, bool nl=true);

#if defined(LEFT_REMOTE)
  SerialPIO ser_;
#endif
  bool left_led_state_, right_led_state_;
  unsigned long last_millis_;
};

#endif // RDISPLAY_H