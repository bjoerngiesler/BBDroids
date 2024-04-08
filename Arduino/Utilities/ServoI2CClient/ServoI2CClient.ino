#include <Wire.h>

uint8_t servos[3];
static const uint8_t I2C_ADDRESS = 0x17;

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  servos[0]++;
  servos[1]++;
  servos[2]++;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(servos, sizeof(servos));
  Wire.endTransmission();

  Serial.println(String("Sent 0x") + String(servos[0], HEX) + " 0x" + String(servos[1], HEX) + " 0x" + String(servos[2], HEX));

  Serial.print("Requesting 3 bytes...");
  uint8_t num = Wire.requestFrom(I2C_ADDRESS, (uint8_t)3);
  Serial.print(String(" receiving ") + num + "...");
  int timeout = 1000;
  while(!Wire.available() && timeout-- > 0) {
    delay(1);
    Serial.print(".");
  }
  if(timeout < 0) {
    Serial.println("Timeout!");
    return;
  }
  uint8_t pos = 0;
  while(Wire.available() && pos<3) {
    servos[pos++] = Wire.read();
  }
  if(pos>=3 && Wire.available()) {
    Serial.println("Huh? Still data available after 3 bytes");
  }
  Serial.println(String("Received 0x") + String(servos[0], HEX) + " 0x" + String(servos[1], HEX) + " 0x" + String(servos[2], HEX));

  delay(10);
}
