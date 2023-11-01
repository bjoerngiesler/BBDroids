#if !defined(BB8STATUSPIXELS_H)
#define BB8STATUSPIXELS_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LibBB.h>
#include <map>

class BB8StatusPixels {
public:
  static BB8StatusPixels statusPixels;

  BB8StatusPixels();

  typedef enum {
    STATUS_OK,
    STATUS_WARN,
    STATUS_FAIL,
    STATUS_INIT,
    STATUS_ACTIVITY
  } Status;

  bool begin();
  bool isAvailable();
  void update();
  bool setPixel(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  bool setPixel(uint16_t n, Status s);
  bool overridePixelUntil(uint16_t n, Status s, uint64_t millisecondTimestamp);
  void linkSubsystem(bb::Subsystem* subsys, uint16_t pixel, bool autoupdate = true);

protected:
  bool available_;
  std::map<uint16_t, std::vector<bb::Subsystem*>> linkedSubsystems_;
  std::map<uint16_t, uint64_t> overrides_;
};

#endif // BB8STATUSPIXELS_H