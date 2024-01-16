#include <Arduino.h>
#include "Config.h"

#include "RDisplay.h"
#include "RemoteInput.h"
#include "IMUFilter.h"

static const int CIRCLE_X = 40;
static const int TOP_CIRCLE_Y = 50;
static const int BOTTOM_CIRCLE_Y = 120;
static const int CIRCLE_RADIUS = 30;
static const int CURSOR_SIZE = 5;

#define SQ(a) ((a)*(a))
#define DEG2RAD(a) ((a)*M_PI/180.0)
#define RAD2DEG(a) ((a)*180.0/M_PI)

RDisplay RDisplay::display;

RDisplay::RDisplay()
#if defined(LEFT_REMOTE)
: ser_(P_DISPLAY_RX, P_DISPLAY_TX) 
#endif // LEFT_REMOTE
{
  name_ = "display";
	description_ = "Display";
	help_ = "";
}

Result RDisplay::initialize() {
#if defined(LEFT_REMOTE)
  pinMode(P_NEOPIXEL, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(P_DISPLAY_RX, INPUT);
  pinMode(P_DISPLAY_TX, OUTPUT);
  ser_.begin(921600);
  ser_.println("");
  while(ser_.available()) ser_.read(); // readEmpty
  if(sendStringAndWaitForOK("logo backlight") == false) {
    Serial.println("Display failure");
    return RES_SUBSYS_COMM_ERROR;
  }
  Serial.println("Display OK");
#endif
  
  return Subsystem::initialize();
}

Result RDisplay::start(ConsoleStream *stream) {
#if defined(LEFT_REMOTE)
  last_millis_ = millis();
  left_led_state_ = false;
  right_led_state_ = false;

  operationStatus_ = RES_OK;
  started_ = true;

  return RES_OK;
#else
  return RES_SUBSYS_HW_DEPENDENCY_MISSING;
#endif
}
	
Result RDisplay::stop(ConsoleStream *stream) {
  Result res = RES_OK;

  if(sendStringAndWaitForOK("logo") == false) res = RES_SUBSYS_COMM_ERROR;

  started_ = false;
  return res;
}
	
Result RDisplay::step() {
  return update();
}

#if 0
Result RDisplay::showScreen(Screen screen) {
  if(screen == curScreen_) return RES_OK;

  if(screen == LOGO_SCREEN) {
    if(sendStringAndWaitForOK("logo backlight") == false) return RES_SUBSYS_COMM_ERROR;
    curScreen_ = screen;
    return RES_OK;
  }

  else if(screen == LEFT_REMOTE_SCREEN) {
    if(sendStringAndWaitForOK("cls") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK("print 3 0 0xffff \"    Left   >\"") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK(String("circle ") + CIRCLE_X + " " + TOP_CIRCLE_Y + " " + CIRCLE_RADIUS + " 0xffff") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK(String("circle ") + CIRCLE_X + " " + BOTTOM_CIRCLE_Y + " " + CIRCLE_RADIUS + " 0xffff") == false) return RES_SUBSYS_COMM_ERROR;
    
    topX_ = CIRCLE_X;
    topY_ = TOP_CIRCLE_Y;
    bottomX_ = CIRCLE_X;
    bottomY_ = BOTTOM_CIRCLE_Y;
    headingX_ = CIRCLE_X;
    headingY_ = BOTTOM_CIRCLE_Y;

    drawCursor(topX_, topY_, 3, WHITE);
    drawCursor(bottomX_, bottomY_, 3, WHITE);

    curScreen_ = screen;
    return RES_OK;
  }

  else if(screen == RIGHT_REMOTE_SCREEN) {
    if(sendStringAndWaitForOK("cls") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK("print 3 0 0xffff \"<  Right   >\"") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK(String("circle ") + CIRCLE_X + " " + TOP_CIRCLE_Y + " " + CIRCLE_RADIUS + " 0xffff") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK(String("circle ") + CIRCLE_X + " " + BOTTOM_CIRCLE_Y + " " + CIRCLE_RADIUS + " 0xffff") == false) return RES_SUBSYS_COMM_ERROR;
    
    topX_ = CIRCLE_X;
    topY_ = TOP_CIRCLE_Y;
    bottomX_ = CIRCLE_X;
    bottomY_ = BOTTOM_CIRCLE_Y;
    headingX_ = CIRCLE_X;
    headingY_ = BOTTOM_CIRCLE_Y;

    curScreen_ = screen;
  }

  else if(screen == DROID_STATUS_SCREEN) {
    if(sendStringAndWaitForOK("cls") == false) return RES_SUBSYS_COMM_ERROR;
    if(sendStringAndWaitForOK("print 3 0 0xffff \"<   Droid\"") == false) return RES_SUBSYS_COMM_ERROR;

    curScreen_ = screen;
  }

  return RES_CMD_FAILURE;
}
#endif

Result RDisplay::update() {
#if 0
  // blink left LED as long as we are not connected, otherwise switch it on
  unsigned long current_millis = millis();
  if(current_millis - last_millis_ > 500) {
    if(connected_) {
      digitalWrite(LED_BUILTIN, LOW);
      left_led_state_ = true;
    }
    else {
      digitalWrite(LED_BUILTIN, left_led_state_);
      left_led_state_ = !left_led_state_;
    }
    last_millis_ = current_millis;
  }

  if(curScreen_ == LEFT_REMOTE_SCREEN) {
    int x, y;

    float length = sqrt(SQ(RemoteInput::input.joyH) + SQ(RemoteInput::input.joyV));
    float angle = atan2(RemoteInput::input.joyV, RemoteInput::input.joyH);
    if(length > 1) length = 1;

    x = cos(angle)*0.9*length*CIRCLE_RADIUS + CIRCLE_X;
    y = TOP_CIRCLE_Y - sin(angle)*0.9*length*CIRCLE_RADIUS;

    //x = RemoteInput::input.joyH * 0.9 * CIRCLE_RADIUS + CIRCLE_X;
    //y = RemoteInput::input.joyV * 0.9 * CIRCLE_RADIUS + TOP_CIRCLE_Y;
    if(x != topX_ || y != topY_) {
      drawCursor(topX_, topY_, CURSOR_SIZE, BLACK);
      topX_ = x; topY_ = y;
      drawCursor(topX_, topY_, CURSOR_SIZE, WHITE);
    }

    if (IMUFilter::imu.available()) {
      float roll, pitch, heading;
      IMUFilter::imu.update();
      IMUFilter::imu.getFilteredEulerAngles(roll, pitch, heading);

      int oldX = headingX_, oldY = headingY_;

      headingX_ = cos(DEG2RAD(heading-90.0)) * CIRCLE_RADIUS/2 + CIRCLE_X;
      headingY_ = sin(DEG2RAD(heading-90.0)) * CIRCLE_RADIUS/2 + BOTTOM_CIRCLE_Y;

      //if(oldX-(int)headingX_ > 0 || oldY-(int)headingY_ > 0) {
        if(sendStringAndWaitForOK(String("line ") + CIRCLE_X + " " + BOTTOM_CIRCLE_Y + " " + oldX + " " + oldY + " 0x0000") == false) return RES_SUBSYS_COMM_ERROR;
        if(sendStringAndWaitForOK(String("line ") + CIRCLE_X + " " + BOTTOM_CIRCLE_Y + " " + (int)headingX_ + " " + (int)headingY_ + " 0xffff") == false) return RES_SUBSYS_COMM_ERROR;
      //}

      oldX = bottomX_; oldY = bottomY_;

      bottomX_ = roll * ((0.9*CIRCLE_RADIUS)/90.0);
      bottomY_ = pitch * ((0.9*CIRCLE_RADIUS)/90.0);
      float length = sqrt(SQ(bottomX_) + SQ(bottomY_));
      if(length > 0.9*CIRCLE_RADIUS) {
        bottomX_ = (int)((float)bottomX_/length);
        bottomY_ = (int)((float)bottomX_/length);
      }

      bottomX_ += CIRCLE_X;
      bottomY_ += BOTTOM_CIRCLE_Y;

      //if(oldX-(int)bottomX_ > 0 || oldY-(int)bottomY_ > 0) {
        drawCursor(oldX, oldY, CURSOR_SIZE, BLACK);
        drawCursor(bottomX_, bottomY_, CURSOR_SIZE, WHITE);
      //}
    } else {
      drawCursor(bottomX_, bottomY_, CURSOR_SIZE, RED);
    }
  }

  else if(curScreen_ == RIGHT_REMOTE_SCREEN) {
    CommandPacket p = lastPacketFromRightRemote_.payload.cmd;

    float x = p.getAxis(0) / 127.0, y = p.getAxis(1) / 127.0;    

    float length = sqrt(SQ(x) + SQ(y));
    float angle = atan2(y, x);
    if(length > 1) length = 1;

    x = cos(angle)*0.9*length*CIRCLE_RADIUS + CIRCLE_X;
    y = TOP_CIRCLE_Y - sin(angle)*0.9*length*CIRCLE_RADIUS;

    if(x != topX_ || y != topY_) {
      drawCursor(topX_, topY_, CURSOR_SIZE, BLACK);
      topX_ = x; topY_ = y;
      drawCursor(topX_, topY_, CURSOR_SIZE, WHITE);
    }

    float roll, pitch, heading;
    roll = ((float)p.getAxis(2))/0.7;
    pitch = ((float)p.getAxis(3))/0.7;
    heading = ((float)p.getAxis(4))/0.7;
    
    int oldX = headingX_, oldY = headingY_;

    headingX_ = cos(DEG2RAD(heading-90.0)) * CIRCLE_RADIUS/2 + CIRCLE_X;
    headingY_ = sin(DEG2RAD(heading-90.0)) * CIRCLE_RADIUS/2 + BOTTOM_CIRCLE_Y;

    //if(oldX-(int)headingX_ > 0 || oldY-(int)headingY_ > 0) {
      if(sendStringAndWaitForOK(String("line ") + CIRCLE_X + " " + BOTTOM_CIRCLE_Y + " " + oldX + " " + oldY + " 0x0000") == false) return RES_SUBSYS_COMM_ERROR;
      if(sendStringAndWaitForOK(String("line ") + CIRCLE_X + " " + BOTTOM_CIRCLE_Y + " " + (int)headingX_ + " " + (int)headingY_ + " 0xffff") == false) return RES_SUBSYS_COMM_ERROR;
    //}

    oldX = bottomX_; oldY = bottomY_;

    bottomX_ = roll * ((0.9*CIRCLE_RADIUS)/90.0);
    bottomY_ = pitch * ((0.9*CIRCLE_RADIUS)/90.0);
    length = sqrt(SQ(bottomX_) + SQ(bottomY_));
    if(length > 0.9*CIRCLE_RADIUS) {
      bottomX_ = (int)((float)bottomX_/length);
      bottomY_ = (int)((float)bottomX_/length);
    }

    bottomX_ += CIRCLE_X;
    bottomY_ += BOTTOM_CIRCLE_Y;

    //if(oldX-(int)bottomX_ > 0 || oldY-(int)bottomY_ > 0) {
      drawCursor(oldX, oldY, CURSOR_SIZE, BLACK);
      drawCursor(bottomX_, bottomY_, CURSOR_SIZE, WHITE);
    //}
  }
#endif
  return RES_OK;
}

void RDisplay::setConnected(bool conn) {
  connected_ = conn;
}

Result RDisplay::drawCursor(int x, int y, int width, uint16_t color) {
  if(sendStringAndWaitForOK(String("line ") + (x-width/2) + " " + y + " " + (x+width/2) + " " + y + " 0x" + String(color, HEX)) == false) return RES_SUBSYS_COMM_ERROR;
  if(sendStringAndWaitForOK(String("line ") + x + " " + (y-width/2) + " " + x + " " + (y+width/2) + " 0x" + String(color, HEX)) == false) return RES_SUBSYS_COMM_ERROR;
  return RES_OK;
}

String RDisplay::sendStringAndWaitForResponse(const String& str, int predelay, bool nl) {
#if defined(LEFT_REMOTE)
  //Serial.println(String("Sending \"") + str + "\"");
  ser_.print(str);
  if(nl) {
    ser_.print("\n");
  }
  ser_.flush();


  if(predelay > 0) delay(predelay);

  String retval;
  if(readString(retval)) {
    return retval;
  }
#endif

  return "";
}
  
bool RDisplay::sendStringAndWaitForOK(const String& str, int predelay, bool nl) {
  String result = sendStringAndWaitForResponse(str, predelay, nl);
  //Serial.println(String("Received \"") + result + "\"");
  if(result.equals("OK.\r\n")) return true;
  return false;
}

bool RDisplay::readString(String& str, unsigned char terminator) {
#if defined(LEFT_REMOTE)
	while(true) {
		int to = 0;
		while(!ser_.available()) {
			delay(1);
			to++;
			if(to >= 1000) {
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
  if(!sendStringAndWaitForOK("cls")) return RES_SUBSYS_COMM_ERROR;
  return RES_OK;
}

Result RDisplay::text(uint8_t x, uint8_t y, uint16_t color, const String& text) {
  String str = String("print ") + x + " " + y + " 0x" + String(color, HEX) + " \"" + text + "\"";
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR; 
  return RES_OK;
}

Result RDisplay::hline(uint8_t x, uint8_t y, uint8_t width, uint16_t color) {
  String str = String("hline ") + x + " " + y + " " + width + " 0x" + String(color, HEX);
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
  return RES_OK;
}

Result RDisplay::rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color, bool filled) {
  String str;
  if(filled) str = String("rectfilled ") + x1 + " " + y1 + " " + x2 + " " + y2 + " " + " 0x" + String(color, HEX);
  else str = String("rect ") + x1 + " " + y1 + " " + x2 + " " + y2 + " " + " 0x" + String(color, HEX);
  if(!sendStringAndWaitForOK(str)) return RES_SUBSYS_COMM_ERROR;
  return RES_OK;
}