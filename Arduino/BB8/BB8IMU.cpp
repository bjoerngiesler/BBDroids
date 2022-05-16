#include "BB8IMU.h"

BB8IMU::BB8IMU() {
  begun = false;
}

bool BB8IMU::begin(uint8_t address) {
  return (begun = lis.begin(address));
}

bool BB8IMU::readVector(int16_t& x, int16_t& y, int16_t& z) {
  if(!begun) return false;
  lis.read();
  x = lis.x;
  y = lis.y;
  z = lis.z;
  return true;
}
