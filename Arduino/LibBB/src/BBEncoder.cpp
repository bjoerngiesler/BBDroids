#include <BBEncoder.h>
#include <BBConsole.h>

#if defined(ARDUINO_CYTRON_MOTION_2350_PRO)
namespace enc {
static const uint8_t NUM_ENC_SLOTS = 10;

struct EncDescr {
  long value = 0;
  uint8_t pinA = 0, pinB = 0;
  uint8_t state = 0;
  bool taken = false;
};

static volatile EncDescr encDescr_[NUM_ENC_SLOTS];

static inline __attribute__((always_inline)) void isr(int i) {  
  if(i >= NUM_ENC_SLOTS) return;
  //noInterrupts();
  uint8_t s = encDescr_[i].state;
  if(digitalRead(encDescr_[i].pinA)) s |= 4;
  if(digitalRead(encDescr_[i].pinB)) s |= 8;
  switch(s) {
  case 0: case 5: case 10: case 15:
    break; // no movement
  case 1: case 7: case 8: case 14:
    encDescr_[i].value++;
    break;
  case 2: case 4: case 11: case 13:
    encDescr_[i].value--;
    break;
  case 3: case 12:
    encDescr_[i].value += 2;
    break;
  default:
    encDescr_[i].value -= 2;
    break;
  }
  encDescr_[i].state = (s >> 2);
  //interrupts();
}

static inline void isr0() {isr(0);}
static inline void isr1() {isr(1);}
static inline void isr2() {isr(2);}
static inline void isr3() {isr(3);}
static inline void isr4() {isr(4);}
static inline void isr5() {isr(5);}
static inline void isr6() {isr(6);}
static inline void isr7() {isr(7);}
static inline void isr8() {isr(8);}
static inline void isr9() {isr(9);}
}; // namespace enc
#endif // ARDUINO_CYTRON_MOTION_2350_PRO

bb::Encoder::Encoder(uint8_t pin_enc_a, uint8_t pin_enc_b, InputMode mode, Unit unit): 
#if !defined(ARDUINO_CYTRON_MOTION_2350_PRO)
  enc_(pin_enc_a, pin_enc_b),
#endif
  filtSpeed_(25, 100, true), 
  filtPos_(25, 100, true) {
  mode_ = mode;
  unit_ = unit;
  mmPT_ = 1.0;
  lastCycleUS_ = micros();
#if !defined(ARDUINO_CYTRON_MOTION_2350_PRO)
  presentPos_ = enc_.read();
#else
  presentPos_ = 0;
  pinEncA_ = pin_enc_a;
  pinEncB_ = pin_enc_b;

  pinMode(pinEncA_, INPUT);
  pinMode(pinEncB_, INPUT);

  for(enc_ = 0; enc::encDescr_[enc_].taken == true; enc_++);
  if(enc_ >= enc::NUM_ENC_SLOTS) return; // Should raise an exception here, but...

  bb::printf("Setting up ISRs for slot %d\n", enc_);
  switch(enc_) {
  case 0:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr0, CHANGE);
    break;
  case 1:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr1, CHANGE);
    break;
  case 2:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr2, CHANGE);
    break;
  case 3:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr3, CHANGE);
    break;
  case 4:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr4, CHANGE);
    break;
  case 5:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr5, CHANGE);
    break;
  case 6:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr6, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr6, CHANGE);
    break;
  case 7:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr7, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr7, CHANGE);
    break;
  case 8:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr8, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr8, CHANGE);
    break;
  case 9:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), enc::isr9, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), enc::isr9, CHANGE);
    break;
  default:
    bb::printf("Encoder ISR table full\n");
  }
  enc::encDescr_[enc_].pinA = pin_enc_a;
  enc::encDescr_[enc_].pinB = pin_enc_b;
  enc::encDescr_[enc_].taken = true;
#endif
}

bb::Encoder::~Encoder() {
#if defined(ARDUINO_CYTRON_MOTION_2350_PRO)
  if(enc_ < enc::NUM_ENC_SLOTS) {
    detachInterrupt(pinEncA_);
    detachInterrupt(pinEncB_);
    enc::encDescr_[enc_].taken = false;
  }
#endif
}

void bb::Encoder::setMode(Encoder::InputMode mode) {
  mode_ = mode;
}

void bb::Encoder::setUnit(Encoder::Unit unit) {
  unit_ = unit;
}

bb::Result bb::Encoder::update() {
#if defined(ARDUINO_CYTRON_MOTION_2350_PRO)
  unsigned long ticks = enc::encDescr_[enc_].value; // FIXME
#else
  unsigned long ticks = enc_.read();
#endif

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

float bb::Encoder::present(bb::Encoder::InputMode mode, bool raw) {
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

  if(unit_ == UNIT_TICKS) {
    return retval;
  } else {
    return retval * mmPT_;
  }
}

  
float bb::Encoder::present() {
  return present(mode_);
}

float bb::Encoder::presentPosition(bool raw) {
  return present(INPUT_POSITION, raw);
}

float bb::Encoder::presentSpeed(bool raw) {
  return present(INPUT_SPEED, raw);  
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
