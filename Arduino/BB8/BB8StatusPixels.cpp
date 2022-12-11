#include "BB8StatusPixels.h"
#include "BB8Config.h"

BB8StatusPixels BB8StatusPixels::statusPixels;

BB8StatusPixels::BB8StatusPixels() {
  available_ = false;
  statusPixel_ = NULL;
}

BB8StatusPixels::~BB8StatusPixels() {
  if(statusPixel_ != NULL) delete statusPixel_;
}

bool BB8StatusPixels::begin() {
  Serial.print("Setting up board NeoPixels... no way to tell if this works, so let's say... ");
  statusPixel_ = new Adafruit_NeoPixel(3, P_STATUS_NEOPIXEL, NEO_GRB + NEO_KHZ800);
  statusPixel_->begin();
  statusPixel_->clear();
  statusPixel_->setPixelColor(STATUSPIXEL_OVERALL, statusPixel_->Color(150, 150, 0));
  statusPixel_->setPixelColor(STATUSPIXEL_NETWORK, statusPixel_->Color(150, 150, 0));
  statusPixel_->setPixelColor(STATUSPIXEL_MOTORS, statusPixel_->Color(150, 150, 0));
  statusPixel_->show();
  available_ = true;
  Serial.println("success.");
  return true;
}

bool BB8StatusPixels::setPixel(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  statusPixel_->setPixelColor(n, statusPixel_->Color(r, g, b));
  statusPixel_->show();
}

bool BB8StatusPixels::setPixel(uint16_t n, Status s) {
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
  }
}

bool BB8StatusPixels::isAvailable() {
  return available_;
}