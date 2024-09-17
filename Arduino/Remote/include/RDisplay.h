#if !defined(RDISPLAY_H)
#define RDISPLAY_H

#include <Adafruit_NeoPixel.h>
#include <LibBB.h>
#if defined(LEFT_REMOTE)
#if !defined(ESP32_REMOTE)
#include <SoftwareSerial.h>
#endif
#endif
#include <vector>

#include "RInput.h"

using namespace bb;

#define TWOBITRGB_TO_COLOR(r,g,b) ((r&0x3)<<4 | (g&0x3)<<2 | (b&0x3))

class RDisplay: public Subsystem {
public:
  static const uint8_t BLACK       = TWOBITRGB_TO_COLOR(0, 0, 0);
  static const uint8_t GREY        = TWOBITRGB_TO_COLOR(1, 1, 1);
  static const uint8_t LIGHTGREY   = TWOBITRGB_TO_COLOR(2, 2, 2);
  static const uint8_t WHITE       = TWOBITRGB_TO_COLOR(3, 3, 3);

  static const uint8_t RED         = TWOBITRGB_TO_COLOR(3, 0, 0);
  static const uint8_t DARKRED     = TWOBITRGB_TO_COLOR(2, 0, 0);
  static const uint8_t DIMRED      = TWOBITRGB_TO_COLOR(1, 0, 0);
  static const uint8_t LIGHTRED1   = TWOBITRGB_TO_COLOR(3, 1, 1);
  static const uint8_t LIGHTRED2   = TWOBITRGB_TO_COLOR(3, 2, 2);

  static const uint8_t GREEN       = TWOBITRGB_TO_COLOR(0, 3, 0);
  static const uint8_t DARKGREEN   = TWOBITRGB_TO_COLOR(0, 2, 0);
  static const uint8_t DIMGREEN    = TWOBITRGB_TO_COLOR(0, 1, 0);
  static const uint8_t LIGHTGREEN1 = TWOBITRGB_TO_COLOR(1, 3, 1);
  static const uint8_t LIGHTGREEN2 = TWOBITRGB_TO_COLOR(2, 3, 2);

  static const uint8_t BLUE        = TWOBITRGB_TO_COLOR(0, 0, 3);
  static const uint8_t DARKBLUE    = TWOBITRGB_TO_COLOR(0, 0, 2);
  static const uint8_t DIMBLUE     = TWOBITRGB_TO_COLOR(0, 0, 1);
  static const uint8_t LIGHTBLUE1  = TWOBITRGB_TO_COLOR(1, 1, 3);
  static const uint8_t LIGHTBLUE2  = TWOBITRGB_TO_COLOR(2, 2, 3);

  static const uint8_t YELLOW      = TWOBITRGB_TO_COLOR(3, 3, 0);
  static const uint8_t DARKYELLOW  = TWOBITRGB_TO_COLOR(2, 2, 0);
  static const uint8_t DIMYELLOW   = TWOBITRGB_TO_COLOR(1, 1, 0);

  static uint8_t RGBtoColor(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xc0)>>2) | ((g & 0xc0)>>4) | ((b & 0xc0)>>4);
  }

  static const uint8_t CHAR_WIDTH = 6;
  static const uint8_t CHAR_HEIGHT = 10;
  static const uint8_t DISPLAY_WIDTH = 80;
  static const uint8_t DISPLAY_HEIGHT = 160;
  static const uint8_t MAIN_X = 0;
  static const uint8_t MAIN_Y = 13;
  static const uint8_t MAIN_WIDTH = DISPLAY_WIDTH;
  static const uint8_t MAIN_HEIGHT = DISPLAY_HEIGHT-26;

  enum WhichLED {
    LED_LEFT,
    LED_RIGHT,
    LED_BOTH
  };

  static RDisplay display;

  virtual Result initialize();
	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();

  Result cls();
  Result text(uint8_t x, uint8_t y, uint8_t color, const String& text);
  Result hline(uint8_t x, uint8_t y, uint8_t width, uint8_t color);
  Result vline(uint8_t x, uint8_t y, uint8_t height, uint8_t color);
  Result line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
  Result rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color, bool filled=false);
  Result circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color, bool filled = false);
  Result plot(uint8_t x, uint8_t y, uint8_t color);

  Result setLED(WhichLED which, uint8_t r, uint8_t g, uint8_t b);
  Result flashLED(WhichLED which, uint8_t iterations, uint8_t millisOn, uint8_t millisOff, uint8_t r, uint8_t g, uint8_t b);
  Result showLEDs();

protected:
  RDisplay();
  virtual ~RDisplay() {}
  bool readString(String& str, unsigned char terminator='\n');

  String sendStringAndWaitForResponse(const String& str, int timeout=0, bool nl=true);
  bool sendStringAndWaitForOK(const String& str, int timeout=1, bool nl=true);
  uint8_t sendBinCommand(const std::vector<uint8_t>& cmd, int timeout=1000, bool waitForResponse=false);

#if defined(LEFT_REMOTE)
#if defined(ESP32_REMOTE)
  HardwareSerial& ser_;
#else
  SerialPIO ser_;
#endif
#endif
  bool left_led_state_, right_led_state_;
  unsigned long last_millis_;
  Adafruit_NeoPixel statusPixels_;
};

#endif // RDISPLAY_H