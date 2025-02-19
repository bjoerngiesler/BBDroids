#include <Wire.h>
#include <Servo.h>
#include <vector>
#include <Adafruit_NeoPixel.h>

static const uint8_t I2C_ADDRESS  = 0x17;

static const uint8_t SERVO_MIN = 0;
static const uint8_t SERVO_MAX = 180;
static const uint8_t SERVO_CENTER = (SERVO_MAX-SERVO_MIN)/2 + SERVO_MIN;

int servoPins[3] = {35, 36, 37};
Servo servo;
int servoSetpoints[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
int servoCurrents[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
uint8_t servoSpeed = 1;
uint8_t cur_reg = 0;

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

#define WIRE Wire1

void writeServos(bool hard) {
  for(int i=0; i<3; i++) {
    if(hard) servoCurrents[i] = servoSetpoints[i];
    else {
      if(servoCurrents[i] < servoSetpoints[i]) {
        servoCurrents[i] += servoSpeed;
        if(servoCurrents[i] > servoSetpoints[i]) {
          servoCurrents[i] = servoSetpoints[i];
        }
      } else if(servoCurrents[i] > servoSetpoints[i]) {
        servoCurrents[i] -= servoSpeed;
        if(servoCurrents[i] < servoSetpoints[i]) {
          servoCurrents[i] = servoSetpoints[i];
        }
      }
    }
    servo.write(servoPins[i], float(servoCurrents[i]));
  }
}

void receiveEvent(int howmany) {
  if(!WIRE.available()) return;
  if(howmany == 3) {
    for(int i=0; i<3; i++) {
      uint8_t sp = WIRE.read();
      servoSetpoints[i] = map(sp, 0, 180, SERVO_MIN, SERVO_MAX);
    }
  }
}

void requestEvent() {
  for(uint8_t i=0; i<3; i++) {
    WIRE.write(map(servoSetpoints[i], SERVO_MIN, SERVO_MAX, 0, 180));
  }
}

void setup() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  pixels.begin();
  pixels.setBrightness(20);
  pixels.fill(0x00ff00);
  pixels.show();
#endif

  Serial.begin(115200);
  Serial.println("Antenna Servos I2C Slave");
  Serial.println("2024 by Bjoern Giesler");
  Serial.println(String("Listening on 0x") + String(I2C_ADDRESS, HEX));
  Serial.println("Listening to i2c...");

  writeServos(true);
  WIRE.onReceive(receiveEvent);
  WIRE.onRequest(requestEvent);
  WIRE.begin(I2C_ADDRESS, SDA1, SCL1, 1000000);
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
  uint8_t sp[3];
  if(words.size() == 1) {
    sp[0] = sp[1] = sp[2] = words[0].toInt();
    Serial.print("Setting all to "); Serial.println(sp[0]);
    for(int i=0; i<3; i++) {
      servoSetpoints[i] = map(sp[0], 0, 180, SERVO_MIN, SERVO_MAX);
    }
  } else if(words.size() == 3) {
    Serial.print("Setting to ");
    for(int i=0; i<3; i++) {
      sp[i] = words[i].toInt();
      Serial.print(sp[i]); Serial.print(" ");
      servoSetpoints[i] = map(sp[i], 0, 180, SERVO_MIN, SERVO_MAX);
    }
    Serial.println();
  } else {
    Serial.println("Enter one int (sets all aerials to the same angle) or 3 ints (one for each aerial).");
    Serial.println("Range is [0..180].");
  }
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

  noInterrupts();
  writeServos(true);
  interrupts();

  delay(2);
}
