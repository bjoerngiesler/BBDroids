#include <Wire.h>

static const uint8_t I2C_ADDRESS  = 0x17;
static const uint8_t REG_SERVO_TOP          = 0x0;
static const uint8_t REG_SERVO_BOTTOM_LEFT  = 0x1;
static const uint8_t REG_SERVO_BOTTOM_RIGHT = 0x2;

static const uint8_t PIN_SERVO_TOP           = 9;
static const uint8_t PIN_SERVO_BOTTOM_LEFT  = 10;
static const uint8_t PIN_SERVO_BOTTOM_RIGHT = 11;

static const uint8_t SERVO_MIN = 130;
static const uint8_t SERVO_MAX = 255;
static const uint8_t SERVO_CENTER = (SERVO_MAX-SERVO_MIN)/2 + SERVO_MIN;

uint8_t servos[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
uint8_t cur_reg = 0;

void writeServos() {
  analogWrite(PIN_SERVO_TOP, servos[REG_SERVO_TOP]);
  analogWrite(PIN_SERVO_BOTTOM_LEFT, servos[REG_SERVO_BOTTOM_LEFT]);
  analogWrite(PIN_SERVO_BOTTOM_RIGHT, servos[REG_SERVO_BOTTOM_RIGHT]);
}

void receiveEvent(int howmany) {
  if(!Wire.available()) return;
  if(howmany == 3) {
    servos[REG_SERVO_TOP] = map(Wire.read(), 0, 255, SERVO_MIN, SERVO_MAX);
    servos[REG_SERVO_BOTTOM_LEFT] = map(Wire.read(), 0, 255, SERVO_MIN, SERVO_MAX);
    servos[REG_SERVO_BOTTOM_RIGHT] = map(Wire.read(), 0, 255, SERVO_MIN, SERVO_MAX);
  }
}

void requestEvent() {
  for(uint8_t i=0; i<3; i++) {
    Wire.write(map(servos[i], SERVO_MIN, SERVO_MAX, 0, 255));
  }
}

void setup() {
  writeServos();
  Serial.begin(115200);
  Serial.println("Antenna Servos I2C Slave");
  Serial.println("2024 by Bjoern Giesler");
  Serial.println(String("Listening on 0x") + String(I2C_ADDRESS, HEX));
  Serial.println(String("Hook up top servo to pin ") + PIN_SERVO_TOP);
  Serial.println(String("Hook up bottom left servo to pin ") + PIN_SERVO_BOTTOM_LEFT);
  Serial.println(String("Hook up bottom right servo to pin ") + PIN_SERVO_BOTTOM_RIGHT);
  Serial.println("Listening to i2c...");
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  noInterrupts();
  writeServos();
  interrupts();
  delay(20);
}
