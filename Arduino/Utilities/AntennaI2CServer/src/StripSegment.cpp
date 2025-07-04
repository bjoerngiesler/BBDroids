#include "StripSegment.h"

void StripSegment::Segment::step() {
  switch(bounce_) {
  case BOUNCE_CONTACT:
    if((vel_ < 0 && pos_-width_/2 <= 0) ||
       (vel_ > 0 && pos_+width_/2 >= 1))
       vel_ = -vel_;
    break;
  case BOUNCE_VANISH:
    if((vel_ < 0 && pos_+width_/2 <= 0) ||
       (vel_ > 0 && pos_-width_/2 >= 1))    
       vel_ = -vel_;
    break;
  default:
    break;
  }
  moveBy(vel_*(millis()-lastmsec_)/1000.0f); 
  lastmsec_ = millis();
}

StripSegment::StripSegment(Adafruit_NeoPixel& strip, uint8_t first, uint8_t last): strip_(strip) {
  if(last > first) {
    first_ = first;
    last_ = last;
    reverse_ = false;
  } else {
    first_ = last;
    last_ = first;
    reverse_ = true;
  }
}

void StripSegment::clear() {
  for(int i=first_; i<=last_; i++) strip_.setPixelColor(i, strip_.Color(0, 0, 0));
}

void StripSegment::setPixel(float position, uint8_t r, uint8_t g, uint8_t b, float intensity) {
  float pixelPos = reverse_ ? last_ - position*(last_-first_) : first_ + position*(last_-first_);

  float p1Frac = pixelPos - floor(pixelPos);
  float p2Frac = ceil(pixelPos) - pixelPos;
  strip_.setPixelColor(constrain(ceil(pixelPos), first_, last_), strip_.Color(r*p1Frac*intensity, g*p1Frac*intensity, b*p1Frac*intensity));
  strip_.setPixelColor(constrain(floor(pixelPos), first_, last_), strip_.Color(r*p2Frac*intensity, g*p2Frac*intensity, b*p2Frac*intensity));
}

void StripSegment::setPixelRange(float pos1, float pos2, uint8_t r, uint8_t g, uint8_t b, float intensity) {
  float pixelPos1 = reverse_ ? last_ - pos1*(last_-first_) : first_ + pos1*(last_-first_);
  float pixelPos2 = reverse_ ? last_ - pos2*(last_-first_) : first_ + pos2*(last_-first_);
  setPixel(pos1, r, g, b, intensity);
  setPixel(pos2, r, g, b, intensity);
  if(reverse_) {
    for(int i=ceil(pixelPos2); i<ceil(pixelPos1); i++) {
      strip_.setPixelColor(constrain(i, first_, last_), strip_.Color(r*intensity, g*intensity, b*intensity));
    }
  } else {
    for(int i=ceil(pixelPos1); i<ceil(pixelPos2); i++) {
      strip_.setPixelColor(constrain(i, first_, last_), strip_.Color(r*intensity, g*intensity, b*intensity));
    }
  }
}

void StripSegment::drawSegment(const StripSegment::Segment& segment) {
  if(!segment.visible()) return;
  setPixelRange(segment.pos()-segment.width()/2, segment.pos()+segment.width()/2, segment.r_, segment.g_, segment.b_, segment.intensity_);
}