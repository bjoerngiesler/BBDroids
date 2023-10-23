#if !defined(BB8STATUSPIXELS_H)
#define BB8STATUSPIXELS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class BB8StatusPixels {
public:
  static BB8StatusPixels statusPixels;

  BB8StatusPixels();

  typedef enum {
    STATUS_OK,
    STATUS_WARN,
    STATUS_FAIL,
    STATUS_INIT
  } Status;

  bool begin();
  bool isAvailable();
  bool setPixel(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  bool setPixel(uint16_t n, Status s);

protected:
  bool available_;
};

#endif // BB8STATUSPIXELS_H