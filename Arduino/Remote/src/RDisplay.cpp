#include <Arduino.h>
#include "Config.h"

#include "RDisplay.h"
#include "RInput.h"
#include "RDSerialInterface.h"

#define BINARY

static const int CIRCLE_X = 40;
static const int TOP_CIRCLE_Y = 50;
static const int BOTTOM_CIRCLE_Y = 120;
static const int CIRCLE_RADIUS = 30;
static const int CURSOR_SIZE = 5;

#define SQ(a) ((a)*(a))
#define DEG2RAD(a) ((a)*M_PI/180.0)
#define RAD2DEG(a) ((a)*180.0/M_PI)

RDisplay RDisplay::display;

RDisplay::RDisplay():
#if defined(LEFT_REMOTE)
#if defined(ARDUINO_ARCH_ESP32)
  ser_(Serial2),
#else
  ser_(P_DISPLAY_RX, P_DISPLAY_TX),
#endif
#endif // LEFT_REMOTE
  statusPixels_(2, P_D_NEOPIXEL, NEO_GRB+NEO_KHZ800)
{
  name_ = "display";
	description_ = "Display";
	help_ = "";
}

Result RDisplay::initialize() {
  pinMode(P_D_NEOPIXEL, OUTPUT);
  statusPixels_.begin();
  setLED(LED_COMM, 0, 0, 0);
  setLED(LED_STATUS, 150, 150, 150);
 
#if defined(LEFT_REMOTE)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif
  
  return Subsystem::initialize();
}

Result RDisplay::start(ConsoleStream *stream) {
#if defined(LEFT_REMOTE)
  last_millis_ = millis();
  left_led_state_ = false;
  right_led_state_ = false;

  ser_.begin(921600);
  while(ser_.available()) ser_.read(); // readEmpty

#if defined(BINARY)

  uint8_t retval = sendBinCommand({rd::CMD_LOGO|0x80, 1}, 100000, true);
  while(ser_.available()) Console::console.printfBroadcast("0x%x\n", ser_.read());
  if(retval != (rd::CMD_NOP|0x80)) {
#else
  if(sendStringAndWaitForOK(String((char)rd::CMD_LOGO)+"1") == false) {
#endif

    Console::console.printfBroadcast("Display failure, got 0x%x instead of 0x%x\n", retval, rd::CMD_NOP|0x80);
    return RES_SUBSYS_COMM_ERROR;
  }
  Console::console.printfBroadcast("Display OK\n");

  operationStatus_ = RES_OK;
  started_ = true;

  return RES_OK;
#else
  return RES_SUBSYS_HW_DEPENDENCY_MISSING;
#endif
}
	
Result RDisplay::stop(ConsoleStream *stream) {
  Result res = RES_OK;

#if defined(BINARY)
  if(sendBinCommand({rd::CMD_LOGO|0x80, 1}, 1, true) != (rd::CMD_NOP|0x80)) 
    res = RES_SUBSYS_COMM_ERROR;
#else
  if(sendStringAndWaitForOK(String((char)rd::CMD_LOGO)+"1") == false) res = RES_SUBSYS_COMM_ERROR;
#endif

  started_ = false;
  return res;
}
	
Result RDisplay::step() {
  return RES_OK;
}

String RDisplay::sendStringAndWaitForResponse(const String& str, int timeout, bool nl) {
#if defined(LEFT_REMOTE)
  ser_.print(str);
  if(nl) {
    ser_.print("\n");
  }
  ser_.flush();

  while(!ser_.available() && timeout>=0) {
    delay(1);
    timeout--;
  }

  if(!ser_.available()) return "";

  String retval;
  if(readString(retval)) {
    return retval;
  }
#endif

  return "";
}
  
bool RDisplay::sendStringAndWaitForOK(const String& str, int timeout, bool nl) {
  String result = sendStringAndWaitForResponse(str, timeout, nl);
  //Console::console.printfBroadcast("\"%s\" ==> \"%s\"", str.c_str(), result.c_str());
  if(result.equals("OK.\r\n")) return true;
  return false;
}

uint8_t RDisplay::sendBinCommand(const std::vector<uint8_t>& cmd, int timeout, bool waitForResponse) {
#if defined(LEFT_REMOTE)
  //Console::console.printfBroadcast("Sending... ");
  for(auto byte: cmd) {
    //Console::console.printfBroadcast("%x ", byte);
    ser_.write(byte);
  }
  //Console::console.printfBroadcast("\n");

  if(!waitForResponse) return rd::CMD_NOP|0x80;
  
  while(!ser_.available() && timeout >= 0) {
    delayMicroseconds(100); 
    timeout--;
  }

  if(!ser_.available()) {
    bb::printf("Timeout waiting for display reply sending ");
    for(auto byte: cmd) {
      bb::printf("%x ", byte);
    }
    bb::printf("\n");
    return 0;
  }

  //bb::printf("Available with %dus remaining.\n", timeout);

  uint8_t retval = 0;
  while(ser_.available()) {
    retval = ser_.read();
    //Console::console.printfBroadcast("Read 0x%x\n", retval);
  }
  return retval;
#endif

  return 0;
}

bool RDisplay::readString(String& str, unsigned char terminator) {
#if defined(LEFT_REMOTE)
	while(true) {
		int to = 0;
		while(!ser_.available()) {
			delay(1);
			to++;
			if(to >= 1000) {
        Console::console.printfBroadcast("Timeout! Read so far: \"%s\"\n", str.c_str());
				return false;
			}
		}
		unsigned char c = (unsigned char)ser_.read();
		str += (char)c;
		if(c == terminator) return true;
	}
#else
  return false;
#endif
}

Result RDisplay::cls() {
#if defined(BINARY)
  if(sendBinCommand({rd::CMD_CLS|0x80}) != (rd::CMD_NOP|0x80)) return RES_SUBSYS_COMM_ERROR;
#else
  if(!sendStringAndWaitForOK(String((char)rd::CMD_CLS))) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}

Result RDisplay::text(uint8_t x, uint8_t y, uint8_t color, const String& text) {
#if defined(BINARY)
  std::vector<uint8_t> vec = {rd::CMD_TEXT|0x80, x, y, color};
  for(int i=0; i<text.length(); i++) vec.push_back(text[i]);
  vec.push_back(0);
  sendBinCommand(vec, 500, true);
#else
  String str = String((char)rd::CMD_TEXT) + x + "," + y + "," + color + ",\"" + text + "\"";
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR; 
#endif
  return RES_OK;
}

Result RDisplay::hline(uint8_t x, uint8_t y, uint8_t width, uint8_t color) {
#if defined(BINARY)
   if(sendBinCommand({rd::CMD_HLINE|0x80, x, y, width, color}) != (rd::CMD_NOP|0x80))
    return RES_SUBSYS_COMM_ERROR;
#else
  String str = String((char)rd::CMD_HLINE) + x + "," + y + "," + width + "," + color;
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}

Result RDisplay::vline(uint8_t x, uint8_t y, uint8_t height, uint8_t color) {
#if defined(BINARY)
   if(sendBinCommand({rd::CMD_VLINE|0x80, x, y, height, color}) != (rd::CMD_NOP|0x80))
    return RES_SUBSYS_COMM_ERROR;
#else
  String str = String((char)rd::CMD_VLINE) + x + "," + y + "," + height + "," + color;
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}

Result RDisplay::line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
#if defined(BINARY)
   if(sendBinCommand({rd::CMD_LINE|0x80, x1, y1, x2, y2, color}) != (rd::CMD_NOP|0x80))
    return RES_SUBSYS_COMM_ERROR;
#else
  String str = String((char)rd::CMD_LINE) + x1 + "," + y1 + "," + x2 + "," + y2 + color;
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}


Result RDisplay::rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color, bool filled) {
  uint8_t cmd = filled ? rd::CMD_FILLEDRECT : rd::CMD_RECT;

#if defined(BINARY)
  //if(filled) Console::console.printfBroadcast("R %d %d %d %d 0x%x\n", x1, y1, x2, y2, color);
  if(sendBinCommand({(uint8_t)(cmd|0x80), x1, y1, x2, y2, color}) != (rd::CMD_NOP|0x80))
    return RES_SUBSYS_COMM_ERROR;
#else
  String str = String((char)cmd) + x1 + "," + y1 + "," + x2 + "," + y2 + "," + color;
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}

Result RDisplay::circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color, bool filled) {
  uint8_t cmd = filled ? rd::CMD_FILLEDCIRCLE : rd::CMD_CIRCLE;

#if defined(BINARY)
  if(sendBinCommand({BINCMD(cmd), x, y, radius, color}) != BINCMD(rd::CMD_NOP))
    return RES_SUBSYS_COMM_ERROR;
#else
  String str = String((char)cmd) + x + "," + y + "," + radius + "," + color;
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}


Result RDisplay::plot(uint8_t x, uint8_t y, uint8_t color) {
#if defined(BINARY)
   if(sendBinCommand({rd::CMD_POINT|0x80, x, y, color}) != (rd::CMD_NOP|0x80))
    return RES_SUBSYS_COMM_ERROR;
#else
  String str = String((char)rd::CMD_POINT) + x + "," + y + "," + color;
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
#endif
  return RES_OK;
}

Result RDisplay::setLED(RDisplay::WhichLED which, uint8_t r, uint8_t g, uint8_t b) {
  if(which == LED_BOTH) {
    statusPixels_.setPixelColor(0, r, g, b);
    statusPixels_.setPixelColor(1, r, g, b);
  }
  else {
    if(which == LED_STATUS) {
      statusPixels_.setPixelColor(0, r, g, b);
    } 
    if(which == LED_COMM) {
      statusPixels_.setPixelColor(1, r, g, b);
    }
  }
  
  statusPixels_.show();
  return RES_OK;
}

Result RDisplay::setLED(WhichLED which, WhatColor color) {
  switch(color) {
    case LED_RED: return setLED(which, 255, 0, 0); break;
    case LED_GREEN: return setLED(which, 0, 255, 0); break;
    case LED_BLUE: return setLED(which, 0, 0, 255); break;
    case LED_YELLOW: return setLED(which, 255, 255, 0); break;
    case LED_WHITE: return setLED(which, 255, 255, 255); break;
    case LED_OFF: default: return setLED(which, 0, 0, 0); break;
  }
  return RES_COMMON_NOT_IN_LIST;
}

Result RDisplay::flashLED(WhichLED which, uint8_t iterations, uint8_t millisOn, uint8_t millisOff, uint8_t r, uint8_t g, uint8_t b) {
  for(int i=0; i<iterations; i++) {
    setLED(which, r, g, b);
    delay(millisOn);
    setLED(which, 0, 0, 0);
    delay(millisOff);
  }
  return RES_OK;
}

void RDisplay::setLEDBrightness(uint8_t brightness) {
  statusPixels_.setBrightness(brightness);
  statusPixels_.show();
}