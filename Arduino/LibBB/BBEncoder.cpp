#if defined(ARDUINO_ARCH_SAMD)

#include <BBEncoder.h>

bb::Encoder::Encoder(uint8_t pin_enc_a, uint8_t pin_enc_b, InputMode mode, Unit unit): 
  enc_(pin_enc_a, pin_enc_b),
  filtSpeed_(25, 100, true), 
  filtPos_(25, 100, true) {
  mode_ = mode;
  unit_ = unit;
  mmPT_ = 1.0;
  lastCycleUS_ = micros();
  presentPos_ = enc_.read();
}

void bb::Encoder::setMode(Encoder::InputMode mode) {
  mode_ = mode;
}

void bb::Encoder::setUnit(Encoder::Unit unit) {
  unit_ = unit;
}

bb::Result bb::Encoder::update() {
  unsigned long ticks = enc_.read();
  lastCycleTicks_ = ticks - presentPos_; // FIXME compensate for wrap?
  presentPos_ = ticks;
  presentPosFiltered_ = filtPos_.filter(presentPos_);

  unsigned long us = micros();
  unsigned long dt;
  if (us < lastCycleUS_) {
    dt = (ULONG_MAX - lastCycleUS_) + us;
  } else {
    dt = us - lastCycleUS_;
  }
  lastCycleUS_ = us;

  presentSpeed_ = ((double)lastCycleTicks_ / (double)dt)*1e6;
  presentSpeedFiltered_ = filtSpeed_.filter(presentSpeed_);

  return RES_OK;
}

float bb::Encoder::present(bb::Encoder::InputMode mode, bb::Encoder::Unit unit, bool raw) {
  float retval;
  switch(mode) {
  case INPUT_SPEED:
    if(raw) retval = presentSpeed_;
    else retval = presentSpeedFiltered_;
    break;

  case INPUT_POSITION:
  default:
    if(raw) retval = presentPos_;
    else retval = presentPosFiltered_;
    break;
  }

  if(unit == UNIT_TICKS) return retval;
  else return retval * mmPT_;
}

  
float bb::Encoder::present() {
  return present(mode_, unit_);
}

float bb::Encoder::presentPosition(Unit unit, bool raw) {
  return present(INPUT_POSITION, unit, raw);
}

float bb::Encoder::presentSpeed(Unit unit, bool raw) {
  return present(INPUT_SPEED, unit, raw);  
}


float bb::Encoder::speedFilterCutoff() {
  return filtSpeed_.cutoff();
}
  
void bb::Encoder::setSpeedFilterCutoff(float co) {
  filtSpeed_.setCutoff(co);
}

float  bb::Encoder::positionFilterCutoff() {
  return filtPos_.cutoff();
}

void bb::Encoder::setPositionFilterCutoff(float co) {
  filtPos_.setCutoff(co);
}


#endif // ARDUINO_ARCH_SAMD
