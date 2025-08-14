#include "BB8StatusPixels.h"
#include "BB8Config.h"

BB8StatusPixels BB8StatusPixels::statusPixels;

Adafruit_NeoPixel statusPixel_(3, P_STATUS_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ball1Pixel_(5, P_BALL1_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ball2Pixel_(5, P_BALL2_NEOPIXEL, NEO_GRB + NEO_KHZ800);

struct RGB {
  uint8_t r, g, b;
};

RGB altColors_[] = {{150, 0, 0}, {0, 150, 0}, {0, 0, 150}, {150, 150, 0}, {150, 0, 150}, {0, 150, 150}};
static const unsigned int numAltColors_ = 6;

BB8StatusPixels::BB8StatusPixels() {
  available_ = false;
}

bool BB8StatusPixels::begin() {
  Serial.print("Setting up board NeoPixels... no way to tell if this works, so let's say... ");
  statusPixel_.begin();
  statusPixel_.clear();
  statusPixel_.setPixelColor(STATUSPIXEL_OVERALL, statusPixel_.Color(0, 0, 0));
  statusPixel_.setPixelColor(STATUSPIXEL_REMOTE, statusPixel_.Color(0, 0, 0));
  statusPixel_.setPixelColor(STATUSPIXEL_MOTORS, statusPixel_.Color(0, 0, 0));
  statusPixel_.show();

  ball1Pixel_.begin();
  ball1Pixel_.clear();
  for(int i=0; i<BALL1_NEOPIXEL_COUNT; i++) {
    int j = i % numAltColors_;
    ball1Pixel_.setPixelColor(i, ball1Pixel_.Color(altColors_[j].r, altColors_[j].g, altColors_[j].b));    
  }
  ball1Pixel_.show();

  ball2Pixel_.begin();
  ball2Pixel_.clear();
  for(int i=0; i<BALL2_NEOPIXEL_COUNT; i++) {
    int j = i % numAltColors_;
    ball2Pixel_.setPixelColor(i, ball2Pixel_.Color(altColors_[j].r, altColors_[j].g, altColors_[j].b));
  }
  ball2Pixel_.show();

  available_ = true;
  Serial.println("success.");
  return true;
}

bool BB8StatusPixels::setPixel(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if(!available_) return false;
  statusPixel_.setPixelColor(n, statusPixel_.Color(r, g, b));
  statusPixel_.show();
  return true;
}

bool BB8StatusPixels::setPixel(uint16_t n, Status s) {
  if(!available_) return false;

  switch(s) {
    case STATUS_OK:
    return setPixel(n, 0, 150, 0);
    break;
    case STATUS_WARN:
    return setPixel(n, 150, 20, 0);
    break;
    case STATUS_FAIL:
    return setPixel(n, 150, 0, 0);
    break;
    case STATUS_INIT:
    return setPixel(n, 150, 150, 150);
    case STATUS_ACTIVITY:
    default:
    return setPixel(n, 0, 0, 0);
  }
  return false;
}

bool BB8StatusPixels::overridePixelUntil(uint16_t n, Status s, uint64_t msTimestamp) {
  setPixel(n, s);
  overrides_[n] = msTimestamp;
  return true;
}


bool BB8StatusPixels::isAvailable() {
  return available_;
}

void BB8StatusPixels::update() {
  unsigned long ms = millis();
  for(auto pair: linkedSubsystems_) {
    if(overrides_.count(pair.first) == 1) {
      if(overrides_[pair.first] > ms) continue; // skip this pixel
      overrides_.erase(pair.first);
    }

    unsigned int numStarted = 0;
    for(auto subsys: pair.second) {
      if(subsys->isStarted()) numStarted++;
    }
    if(numStarted == pair.second.size()) {
      setPixel(pair.first, BB8StatusPixels::STATUS_OK);
    } else if(numStarted != 0) {
      setPixel(pair.first, BB8StatusPixels::STATUS_WARN);
    } else {
      setPixel(pair.first, BB8StatusPixels::STATUS_FAIL);
    }
  }
}

void BB8StatusPixels::linkSubsystem(bb::Subsystem* subsys, uint16_t pixel, bool autoupdate) {
  if(linkedSubsystems_.count(pixel) == 0) {
    linkedSubsystems_[pixel] = std::vector<bb::Subsystem*>();
  }
  linkedSubsystems_[pixel].push_back(subsys);
  if(autoupdate) update();
}
