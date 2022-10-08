#if !defined(BB8IMU_H)
#define BB8IMU_H

#include <Adafruit_LIS3DH.h>

class BB8IMU {
public:
  BB8IMU();
  bool begin(uint8_t address);
  bool readVector(int16_t& x, int16_t& y, int16_t& z);
  void printStats(const arduino::String& prefix = "");
private:  
  bool begun;
  uint8_t address;
  Adafruit_LIS3DH lis;
};

#endif // BB8IMU_H
