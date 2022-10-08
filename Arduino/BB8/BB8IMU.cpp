#include "BB8IMU.h"

BB8IMU::BB8IMU() {
  begun = false;
}

bool BB8IMU::begin(uint8_t address) {
  if(!(begun = lis.begin(address))) return false;

  return true;
}

bool BB8IMU::readVector(int16_t& x, int16_t& y, int16_t& z) {
  if(!begun) return false;
  lis.read();
  x = lis.x;
  y = lis.y;
  z = lis.z;
  return true;
}

void BB8IMU::printStats(const String& prefix) {
  Serial.print(prefix); 
  Serial.print("Range "); Serial.print(2 << lis.getRange());
  Serial.print("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print(", data rate: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.print("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.print("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.print("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.print("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.print("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.print("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.print("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.print("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.print("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.print("16 Khz Low Power"); break;
  }
  Serial.println();
}
