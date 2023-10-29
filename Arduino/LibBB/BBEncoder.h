#if !defined(BBENCODER_H)
#define BBENCODER_H

#if defined(ARDUINO_ARCH_SAMD) // FIXME - get rid of dependency of Arduino Encoder class.

#include <Encoder.h>
#include <BBControllers.h>
#include <BBLowPassFilter.h>
#include <limits.h>
#include <math.h>

namespace bb {
class Encoder: public ControlInput {
public:
  enum InputMode {
    INPUT_SPEED    = 0,
    INPUT_POSITION = 1
  };

  enum Unit {
    UNIT_MILLIMETERS = 0,
    UNIT_TICKS       = 1
  };

  Encoder(uint8_t pin_enc_a, uint8_t pin_enc_b, InputMode mode = INPUT_POSITION, Unit unit = UNIT_TICKS);
  
  void setMode(InputMode mode);
  void setUnit(Unit unit);
  InputMode mode() { return mode_; }
  Unit unit() { return unit_; }
  virtual float present();
  virtual float present(InputMode mode, Unit unit, bool raw = false);
  virtual float presentPosition(Unit unit = UNIT_TICKS, bool raw = false);
  virtual float presentSpeed(Unit unit = UNIT_TICKS, bool raw = false);
  virtual Result update();

  float speedFilterCutoff();
  void setSpeedFilterCutoff(float co);

  float positionFilterCutoff();
  void setPositionFilterCutoff(float co);

  void setMillimetersPerTick(float mmPT) { mmPT_ = mmPT; }

  float controlGain() { if(unit_ == UNIT_TICKS) return 1.0f; else return 1/mmPT_; }

protected:
  InputMode mode_;
  Unit unit_;
  ::Encoder enc_; // FIXME -- since this requires SAMD, possibly replace by own encoder handling?
  bb::LowPassFilter filtSpeed_, filtPos_;

  float mmPT_;
  long lastCycleTicks_;
  unsigned long lastCycleUS_;
  long presentPos_;    // internally always in encoder ticks
  long presentPosFiltered_;
  float presentSpeed_; // internally always in encoder ticks per second
  float presentSpeedFiltered_;
};
}

#endif // ARDUINO_ARCH_SAMD


#endif //BBENCODER_H