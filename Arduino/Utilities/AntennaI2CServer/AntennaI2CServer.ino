#include <Wire.h>

static const uint8_t I2C_ADDRESS  = 0x17;

static const uint8_t SERVO_MIN = 130;
static const uint8_t SERVO_MAX = 255;
static const uint8_t SERVO_CENTER = (SERVO_MAX-SERVO_MIN)/2 + SERVO_MIN;

int servoPins[3] = {9, 10, 11};
int servoSetpoints[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
int servoCurrents[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
uint8_t servoSpeed = 1;
uint8_t cur_reg = 0;

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
    analogWrite(servoPins[i], servoCurrents[i]);
  }
}

void receiveEvent(int howmany) {
  if(!Wire.available()) return;
  if(howmany == 3) {
    for(int i=0; i<3; i++) {
      servoSetpoints[i] = map(Wire.read(), 0, 255, SERVO_MIN, SERVO_MAX);
    }
  }
}

void requestEvent() {
  for(uint8_t i=0; i<3; i++) {
    Wire.write(map(servoSetpoints[i], SERVO_MIN, SERVO_MAX, 0, 255));
  }
}

void setup() {
  writeServos(true);
  Serial.begin(115200);
  Serial.println("Antenna Servos I2C Slave");
  Serial.println("2024 by Bjoern Giesler");
  Serial.println(String("Listening on 0x") + String(I2C_ADDRESS, HEX));
  Serial.println("Listening to i2c...");
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  if(Serial.available()) {
    String str = Serial.readStringUntil('\n');
    uint8_t pos = str.toInt();
    Serial.println(String("Setting to ") + pos);
    for(int i=0; i<3; i++) {
      servoSetpoints[i] = map(pos, 0, 255, SERVO_MIN, SERVO_MAX);
    }
  }

  noInterrupts();
  writeServos(false);
  interrupts();

  delay(2);
}
