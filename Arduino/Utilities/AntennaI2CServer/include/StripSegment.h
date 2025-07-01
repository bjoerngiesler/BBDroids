#if !defined(STRIPSEGMENT_H)
#define STRIPSEGMENT_H

#include <Adafruit_NeoPixel.h>

/*
  \brief Class to address parts of a NeoPixel strip

  This class represents a segment (or the whole) of an Adafruit NeoPixel strip, and allows addressing
  in a programmer-friendly manner with float positions ranging from 0 (marking the first pixel in the
  strip) to 1 (marking the last pixel in the strip), interpolating or non-interpolating. This allows
  for easy usage as progress bars, running / moving light accents, or similar.
*/
class StripSegment {
public:
  StripSegment(Adafruit_NeoPixel& strip, uint8_t first, uint8_t last);

  /*
    \brief Animate-able segment of a strip

    This represents a part of a StripSegment of a given color and intensity that can be positioned, moved around,
    and has an idea of animateable motion given in position units per second, so calling its step() method causes
    the segment to move at a given velocity from one end of the strip to another. There is also a notion of
    visibility, as moving the segment at constant velocity will move it out of the displayable range at some point.
  */
  class Segment {
    friend class StripSegment;
  public:
    //! Bounce governs what the segment does at the end of the strip: Continue on, bounce back when it makes contact or when it vanishes from view.
    enum BounceMode {
      BOUNCE_OFF,
      BOUNCE_CONTACT,
      BOUNCE_VANISH
    };

    Segment(float pos, float width, float vel, uint8_t r, uint8_t g, uint8_t b, float intensity=1.0) {
      pos_ = pos; width_ = width; vel_ = vel;
      r_ = r; g_ = g; b_ = b; 
      intensity_ = intensity;
      bounce_ = BOUNCE_OFF;
      lastmsec_ = millis();
    }

    float pos() const { return pos_; }
    float width() const { return width_; }
    float vel() const { return vel_; }
    void moveBy(float delta) { pos_ += delta; }
    void moveTo(float pos) { pos_ = pos; }
    void setWidth(float w) { width_ = w; }
    void setVel(float v) { vel_ = v; } 
    void setColor(uint8_t r, uint8_t g, uint8_t b, float intensity=1.0) { r_ = r; g_ = g; b_ = b; intensity_ = intensity;}
    bool visible() const { return pos_+width_/2 >= 0 && pos_-width_/2 <= 1; }
    bool fullyVisible() const { return pos_-width_/2 >= 0 && pos_+width_/2 <= 1; }
    void setBounce(BounceMode bounce) { bounce_ = bounce; }
    BounceMode bounce() const { return bounce_; }
    void step();
  protected:
    float pos_, width_, vel_;
    BounceMode bounce_;
    uint8_t r_, g_, b_;
    float intensity_;
    unsigned long lastmsec_;
  };

  void clear();
  void setPixel(float pos, uint8_t r, uint8_t g, uint8_t b, float intensity=1.0);
  void setPixelRange(float pos1, float pos2, uint8_t r, uint8_t g, uint8_t b, float intensity=1.0);
  void drawSegment(const Segment& segment);

protected:
  Adafruit_NeoPixel& strip_;
  bool reverse_;
  uint8_t first_, last_;
};

#endif // STRIPSEGMENT_H