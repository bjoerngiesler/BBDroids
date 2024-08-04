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

class RDrawable {
public:
  RDrawable() { 
    needsFullRedraw_ = needsContentsRedraw_ = true; 
    x_ = y_ = 1;
    width_ = height_ = 10;
  }
  void setPosition(uint8_t x, uint8_t y) { x_ = x; y_ = y; }
  void setSize(uint8_t w, uint8_t h) { width_ = w; height_ = h; setNeedsFullRedraw(); setNeedsCls(); }
  virtual Result draw(ConsoleStream* stream = NULL) = 0;
  void setNeedsCls(bool needs = true) { needsCls_ = needs; }
  void setNeedsFullRedraw(bool needs = true) { needsFullRedraw_ = needs; if(needs) needsContentsRedraw_ = true; }
  void setNeedsContentsRedraw(bool needs = true) { needsContentsRedraw_ = needs; }
protected:
  bool needsCls_, needsFullRedraw_, needsContentsRedraw_;
  uint8_t x_, y_;
  uint8_t width_, height_;
};

class RDisplay: public Subsystem {
public:
  static const uint16_t BLACK = 0x0000;
  static const uint16_t WHITE = 0xffff;
  static const uint16_t RED   = 0xF800;
  static const uint16_t GREEN = 0x0400;
  static const uint16_t BLUE  = 0x001F;
  static const uint16_t YELLOW = 0xFFE0;
  static const uint16_t DARKGREY = 0xAD55;
  static const uint16_t DIMGRAY = 0x6B4D;
  static const uint16_t DARKBLUE = 0x0011;
  static const uint16_t LIGHTGREY = 0xD69A;
  static const uint16_t LIGHTSTEELBLUE = 0xB63B;

  static const uint8_t CHAR_WIDTH = 6;
  static const uint8_t CHAR_HEIGHT = 10;
  static const uint8_t DISPLAY_WIDTH = 80;
  static const uint8_t DISPLAY_HEIGHT = 160;

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
  Result text(uint8_t x, uint8_t y, uint16_t color, const String& text);
  Result hline(uint8_t x, uint8_t y, uint8_t width, uint16_t color);
  Result vline(uint8_t x, uint8_t y, uint8_t height, uint16_t color);
  Result line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
  Result rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color, bool filled=false);
  Result plot(uint8_t x, uint8_t y, uint16_t color);

  Result setLED(WhichLED which, uint8_t r, uint8_t g, uint8_t b);
  Result flashLED(WhichLED which, uint8_t iterations, uint8_t millisOn, uint8_t millisOff, uint8_t r, uint8_t g, uint8_t b);
  Result showLEDs();

protected:
  RDisplay();
  virtual ~RDisplay() {}
  bool readString(String& str, unsigned char terminator='\n');

  String sendStringAndWaitForResponse(const String& str, int predelay=0, bool nl=true);
  bool sendStringAndWaitForOK(const String& str, int predelay=0, bool nl=true);

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