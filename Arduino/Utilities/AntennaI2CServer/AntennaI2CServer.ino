#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

static const uint8_t I2C_ADDRESS  = 0x17;

static const uint8_t SERVO_MIN = 0;
static const uint8_t SERVO_MAX = 180;
static const uint8_t SERVO_CENTER = (SERVO_MAX-SERVO_MIN)/2 + SERVO_MIN;

int servoPins[3] = {35, 36, 37};
Servo servos[3];
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
    servos[i].write(servoPins[i], float(servoCurrents[i]));
  }
}

void receiveEvent(int howmany) {
  if(!WIRE.available()) return;
  if(howmany == 3) {
    for(int i=0; i<3; i++) {
      servoSetpoints[i] = map(Wire.read(), 0, 180, SERVO_MIN, SERVO_MAX);
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

  for(int i=0; i<3; i++) servos[i].attach(servoPins[i]);
  writeServos(true);
  WIRE.begin(I2C_ADDRESS, SDA1, SCL1, 100000);
  WIRE.onReceive(receiveEvent);
  WIRE.onRequest(requestEvent);
}

void loop() {
  if(Serial.available()) {
    String str = Serial.readStringUntil('\n');
    uint8_t pos = str.toInt();
    Serial.println(String("Setting to ") + pos);
    for(int i=0; i<3; i++) {
      servoSetpoints[i] = map(pos, 0, 180, SERVO_MIN, SERVO_MAX);
    }
  }

  noInterrupts();
  writeServos(false);
  interrupts();

  delay(2);
}
