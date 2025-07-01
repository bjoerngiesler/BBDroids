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

StripSegment::Segment lefteye(.6, 0.5, 0, 255, 255, 255, 1.0);
StripSegment::Segment righteye(.6, 0.5, 0, 255, 255, 255, 1.0);
std::deque<StripSegment::Segment> statusSegments;

using namespace antenna;

static Parameters parameters;
static uint8_t *parametersPtr = (uint8_t*)&parameters;
static uint8_t lastAddress = 0;
static uint8_t lastControl;

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
}

void updateControl() {
  // remove all non-visible items
  uint8_t i = 0;
  while(i < statusSegments.size()) {
    statusSegments[i].step();
    if(statusSegments[i].visible() == false) {
      Serial.printf("Removing segment %d because it is invisible\n", i);
      statusSegments.pop_front();
    }
    else i++;
  }

  if(parameters.control == lastControl) return;

  // remove all bouncing items
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
    Serial.printf("    help              -- print this help text\n");
    Serial.printf("    servo [1|2|3] VAL -- set servo 1, 2, 3, or all to VAL\n");
    Serial.printf("    eye [l|r|b] VAL   -- set left eye, right eye, or both to VAL\n");
    Serial.printf("    control VAL       -- set control to VAL\n");
    return;
  }

  if(words[0] == "eye") {
    bool left=false, right=false;
    uint8_t eyeval;
    if(words.size() == 2) {
      left = true;
      right = true;
      eyeval = words[1].toInt();
    } else if(words.size() == 3) {
      if(words[1] == "l" || words[1] == "left") left = true;
      else if(words[1] == "r" || words[1] == "right") right = true;
      else if(words[1] == "b" || words[1] == "both") left = right = true;
      else {
        Serial.printf("Unknown value \"%s\" for eye index\n", words[1].c_str());
        return;
      }
      eyeval = words[2].toInt();
    } else {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
    }
    if(left) {
      Serial.printf("Setting left eye to %d\n", eyeval);
      parameters.eyes[0] = eyeval;
    }
    if(right) {
      Serial.printf("Setting right eye to %d\n", eyeval);
      parameters.eyes[1] = eyeval;
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

  if(words[0] == "control") {
    if(words.size() != 2) {
      Serial.printf("Wrong number of arguments to \"%s\"\n", words[0].c_str());
      return;
    }
    parameters.control = words[1].toInt();
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
  s1.drawSegment(lefteye);
  s3.drawSegment(righteye);
  for(auto& s: statusSegments) s2.drawSegment(s);
  strip.show();

  delay(40);
}
