#include <Wire.h>
#include <Servo.h>
#include <vector>
#include <deque>
#include <Adafruit_NeoPixel.h>
#include "StripSegment.h"
#include "Config.h"
#include "AntennaI2CServer.h"

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
StripSegment s1(strip, 0, 16), s2(strip, 33, 17), s3(strip, 34, 50);

StripSegment::Segment eyes[2] = { StripSegment::Segment(.6, 0.5, 0, 255, 255, 255, 1.0), StripSegment::Segment(.6, 0.5, 0, 255, 255, 255, 1.0) };
std::deque<StripSegment::Segment> statusSegments;

using namespace antenna;

static Parameters parameters;
static uint8_t *parametersPtr = (uint8_t*)&parameters;
static uint8_t lastAddress = 0;
static uint8_t lastControl = 0;
static uint8_t autoblink = 0;
static uint8_t blinkDuration = 0;
static unsigned long nextBlink;

int servoPins[3] = {35, 36, 37};
Servo servo;
int servoCurrents[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
uint8_t servoSpeed = 1;
uint8_t cur_reg = 0;

#define WIRE Wire1

void updateServos(bool hard) {
  for(int i=0; i<3; i++) {
    if(hard) servoCurrents[i] = parameters.servoSetpoints[i];
    else {
      if(servoCurrents[i] < parameters.servoSetpoints[i]) {
        servoCurrents[i] += servoSpeed;
        if(servoCurrents[i] > parameters.servoSetpoints[i]) {
          servoCurrents[i] = parameters.servoSetpoints[i];
        }
      } else if(servoCurrents[i] > parameters.servoSetpoints[i]) {
        servoCurrents[i] -= servoSpeed;
        if(servoCurrents[i] < parameters.servoSetpoints[i]) {
          servoCurrents[i] = parameters.servoSetpoints[i];
        }
      }
    }
    servo.write(servoPins[i], float(servoCurrents[i]));
  }
}

void updateEyes() {
  for(int i=0; i<2; i++) {
    switch(parameters.eyes[i] & EYE_COLOR_MASK) {
    case EYE_COLOR_OFF: 
      eyes[i].setColor(0, 0, 0, 0);
      break;
    case EYE_COLOR_WHITE:
      eyes[i].setColor(255, 255, 255, 1.0);
      break;
    case EYE_COLOR_RED:
      eyes[i].setColor(255, 0, 0, 1.0);
      break;
    case EYE_COLOR_BLUE:
      eyes[i].setColor(0, 0, 255, 1.0);
      break;
    }

    float width = ((parameters.eyes[i] & EYE_SIZE_MASK) >> EYE_SIZE_SHIFT) / 31.0f;
    float pos = parameters.eyePos[i] / 255.0f;
    eyes[i].moveTo(pos);
    
    if(millis() > nextBlink && autoblink != 0 && blinkDuration > 0) {
      width = 0.03;
      blinkDuration--;
    }

    eyes[i].setWidth(width);
  }
  if(millis() > nextBlink && autoblink != 0 && blinkDuration == 0) {
    nextBlink = millis() + 2000*autoblink + random(4000);
    blinkDuration = 8;
  }
}

void updateControl() {
  // Handle status display

  // Move all segments. Also remove all non-visible segments.
  uint8_t i = 0;
  while(i < statusSegments.size()) {
    statusSegments[i].step();
    if(statusSegments[i].visible() == false) {
      Serial.printf("Removing segment %d because it is invisible\n", i);
      statusSegments.pop_front();
    }
    else i++;
  }

  if((parameters.control & CONTROL_STRIP_MASK) != (lastControl & CONTROL_STRIP_MASK)) {
    i=0;
    while(i < statusSegments.size()) {
      if(statusSegments[i].bounce() != StripSegment::Segment::BOUNCE_OFF) {
        statusSegments[i].setBounce(StripSegment::Segment::BOUNCE_OFF);
      }
      else i++;
    }

    lastControl = parameters.control;
    StripSegment::Segment status(1.1, 0.2, -0.5, 255, 255, 255, 1.0);
    status.setBounce(status.BOUNCE_OFF);
    switch(parameters.control & CONTROL_STRIP_MASK) {
    case CONTROL_STRIP_OFF:
      status.setColor(0, 0, 0);
      break;
    case CONTROL_STRIP_SELFTEST:
      status.setColor(255, 255, 255);
      status.setBounce(status.BOUNCE_CONTACT);
      break;
    case CONTROL_STRIP_OK:
      status.setColor(0, 255, 0);
      break;
    case CONTROL_STRIP_DEGRADED:
      status.setColor(255, 255, 0);
      break;
    case CONTROL_STRIP_VEL:
      status.setColor(0, 255, 0);
      break;
    case CONTROL_STRIP_AUTO_POS:
      status.setColor(0, 0, 255);
      break;
    case CONTROL_STRIP_MAN_POS:
      status.setColor(255, 255, 0);
      break;
    case CONTROL_STRIP_ERROR:
    default:
      status.setColor(255, 0, 0);
      status.setBounce(status.BOUNCE_CONTACT);
      break;
    }

    statusSegments.push_back(status);
  }

  // Handle autoblink
  if((lastControl & CONTROL_AUTOBLINK_MASK) != (parameters.control & CONTROL_AUTOBLINK_MASK)) {
    autoblink = (parameters.control & CONTROL_AUTOBLINK_MASK) >> CONTROL_AUTOBLINK_SHIFT;
  }

  // Handle brightness
  if((lastControl & CONTROL_BRIGHTNESS_MASK) != (parameters.control & CONTROL_BRIGHTNESS_MASK)) {
    uint8_t brightness = map((parameters.control & CONTROL_BRIGHTNESS_MASK) >> CONTROL_BRIGHTNESS_SHIFT, 0x0, 0x7, 0, 255);
    strip.setBrightness(brightness);
  }

  lastControl = parameters.control;
}

void receiveEvent(int howmany) {
  if(howmany > sizeof(parameters)+1) {
    Serial.printf("Receive event: Too many bytes (%d, max %d)\n", howmany, sizeof(parameters)+1);
    return;
  }

  lastAddress = WIRE.read();
  for(uint8_t i=lastAddress; i<lastAddress+howmany-1; i++) {
    parametersPtr[i] = WIRE.read();
  }
}

void requestEvent() {
  WIRE.write(parametersPtr[lastAddress]);
}

void setup() {
  strip.begin();
  strip.show();
  strip.setBrightness(50);

  Serial.begin(115200);
  Serial.println("Antenna Servos I2C Slave");
  Serial.println("2024 by Bjoern Giesler");
  Serial.println(String("Listening on 0x") + String(I2C_ADDRESS, HEX));
  Serial.println("Listening to i2c...");

  updateServos(true);
  WIRE.onReceive(receiveEvent);
  WIRE.onRequest(requestEvent);
  WIRE.begin(I2C_ADDRESS, SDA, SCL, 1000000);
}

std::vector<String> split(const String& str) {
  std::vector<String> words;
  unsigned int start = 0, end = 0;
  bool quotes = false;

  while(end < str.length()) {
    if(quotes == true) {
      if(str[end] != '"') {
        end++;
        if(end == str.length()) {
          String substr = str.substring(start, end);
		  substr.trim();
          if(substr.length()) words.push_back(substr);
        }
      } else if(end >= start) {
        quotes = false;
        String substr = str.substring(start, end);
		substr.trim();
        if(substr.length()) words.push_back(substr);
        start = end+1;
        end = end+1;
      }
    } else {
      if(str[end] == '"') {
        quotes = true;
        start++;
        end++;
        continue;
      }
  
      if(str[end] != ' ') {
        end++;
        if(end == str.length()) {
          String substr = str.substring(start, end);
		      substr.trim();
          if(substr.length()) words.push_back(substr);
        }
      } else if(end >= start) {
        String substr = str.substring(start, end);
		    substr.trim();
        if(substr.length()) words.push_back(substr);
        start = end+1;
        end = end+1;
      } 
    }
  }

  return words;
}

void handleCommand(const std::vector<String>& words) {
  if(words.size() == 0) return;

  if(words[0] == "help") {
    if(words.size() > 1) Serial.printf("Invalid number of arguments\n");
    Serial.printf("Commands\n");
    Serial.printf("    help                 -- print this help text\n");
    Serial.printf("    servo [1|2|3] VAL    -- set servo 1, 2, 3, or all to VAL\n");
    Serial.printf("    eye_col [l|r|b] VAL  -- set left, right, or both eye colors to VAL (0..3)\n");
    Serial.printf("    eye_size [l|r|b] VAL -- set left, right, or both eye sizes to VAL (0..31)\n");
    Serial.printf("    eye_pos [l|r|b] VAL  -- set left eye, right eye, or both to VAL (0..255)\n");
    Serial.printf("    status VAL           -- set status to VAL (0..7)\n");
    Serial.printf("    autoblink VAL        -- set autoblink to VAL (0=off..3=most frequent)\n");
    Serial.printf("    brightness VAL       -- set brightness to VAL (0=off..7)\n");
    return;
  }

  if(words[0] == "eye_col") {
    bool left=false, right=false;
    uint8_t val;
    if(words.size() == 2) {
      left = true;
      right = true;
      val = words[1].toInt();
    } else if(words.size() == 3) {
      if(words[1] == "l" || words[1] == "left") left = true;
      else if(words[1] == "r" || words[1] == "right") right = true;
      else if(words[1] == "b" || words[1] == "both") left = right = true;
      else {
        Serial.printf("Unknown value \"%s\" for eye index\n", words[1].c_str());
        return;
      }
      val = words[2].toInt();
    } else {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
    }
    if(left) {
      parameters.eyes[0] = (parameters.eyes[0] & ~EYE_COLOR_MASK) | (val << EYE_COLOR_SHIFT);
    }
    if(right) {
      parameters.eyes[1] = (parameters.eyes[1] & ~EYE_COLOR_MASK) | (val << EYE_COLOR_SHIFT);
    }
    return;
  }

  if(words[0] == "eye_size") {
    bool left=false, right=false;
    uint8_t val;
    if(words.size() == 2) {
      left = true;
      right = true;
      val = words[1].toInt();
    } else if(words.size() == 3) {
      if(words[1] == "l" || words[1] == "left") left = true;
      else if(words[1] == "r" || words[1] == "right") right = true;
      else if(words[1] == "b" || words[1] == "both") left = right = true;
      else {
        Serial.printf("Unknown value \"%s\" for eye index\n", words[1].c_str());
        return;
      }
      val = words[2].toInt();
    } else {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
    }
    if(left) {
      parameters.eyes[0] = (parameters.eyes[0] & ~EYE_SIZE_MASK) | (val << EYE_SIZE_SHIFT);
    }
    if(right) {
      parameters.eyes[1] = (parameters.eyes[0] & ~EYE_SIZE_MASK) | (val << EYE_SIZE_SHIFT);
    }
    return;
  }
  if(words[0] == "eye_pos") {
    bool left=false, right=false;
    uint8_t val;
    if(words.size() == 2) {
      left = true;
      right = true;
      val = words[1].toInt();
    } else if(words.size() == 3) {
      if(words[1] == "l" || words[1] == "left") left = true;
      else if(words[1] == "r" || words[1] == "right") right = true;
      else if(words[1] == "b" || words[1] == "both") left = right = true;
      else {
        Serial.printf("Unknown value \"%s\" for eye index\n", words[1].c_str());
        return;
      }
      val = words[2].toInt();
    } else {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
    }
    if(left) {
      parameters.eyePos[0] = val;
    }
    if(right) {
      parameters.eyePos[1] = val;
    }
    return;
  }

  if(words[0] == "servo") {
    bool set[3] = {false, false, false};
    uint8_t servoval;
    if(words.size() == 2) {
      for(int i=0; i<3; i++) set[i] = true;
      servoval = words[1].toInt();
      if(servoval > 180) servoval = 180;
    } else if(words.size() == 3) {
      if(words[1] == "1") set[1] = true;
      else if(words[1] == "2") set[2] = true;
      else if(words[1] == "3") set[3] = true;
      else {
        Serial.printf("Unknown value \"%s\" for servo index\n", words[1].c_str());
        return;
      }
      servoval = words[2].toInt();
    } else {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
    }
    for(int i=0; i<3; i++) {
      if(set[i] == true) parameters.servoSetpoints[i] = servoval;
    }
    return;
  }

  if(words[0] == "status") {
    if(words.size() != 2) {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
      return;
    }
    uint8_t val = words[1].toInt();
    parameters.control = (parameters.control & ~CONTROL_STRIP_MASK) | (val << CONTROL_STRIP_SHIFT);
    return;
  }

  if(words[0] == "autoblink") {
    if(words.size() != 2) {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
      return;
    }
    uint8_t val = words[1].toInt();
    parameters.control = (parameters.control & ~CONTROL_AUTOBLINK_MASK) | (val << CONTROL_AUTOBLINK_SHIFT);
    return;
  }

  if(words[0] == "brightness") {
    if(words.size() != 2) {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
      return;
    }
    uint8_t val = words[1].toInt();
    parameters.control = (parameters.control & ~CONTROL_BRIGHTNESS_MASK) | (val << CONTROL_BRIGHTNESS_SHIFT);
    return;
  }

  Serial.printf("Unknown command \"%s\"\n", words[0].c_str());
}

static String cmd;

void loop() {
  while(Serial.available()) {
    char c = (char)Serial.read();
    Serial.print(c);
    cmd = cmd + c;
    if(cmd.endsWith("\n")) {
      handleCommand(split(cmd));
      cmd = "";
    }
  }

  updateServos(true);
  updateEyes();
  updateControl();

  s1.clear(); s2.clear(); s3.clear();
  s1.drawSegment(eyes[0]);
  s3.drawSegment(eyes[1]);
  for(auto& s: statusSegments) s2.drawSegment(s);
  strip.show();

  delay(5);
}
