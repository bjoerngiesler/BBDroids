#include <BBEncoder.h>
#include <BBConsole.h>

static const uint8_t NUM_ENC_SLOTS = 10;

struct EncDescr {
  long value = 0;
  uint8_t pinA = 0, pinB = 0;
  uint8_t state = 0;
  bool taken = false;
};

static volatile EncDescr encDescr_[NUM_ENC_SLOTS];

static void isr(int i) {  
  if(i >= NUM_ENC_SLOTS) return;
  noInterrupts();
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
  interrupts();
}

static void isr0() {isr(0);}
static void isr1() {isr(1);}
static void isr2() {isr(2);}
static void isr3() {isr(3);}
static void isr4() {isr(4);}
static void isr5() {isr(5);}
static void isr6() {isr(6);}
static void isr7() {isr(7);}
static void isr8() {isr(8);}
static void isr9() {isr(9);}

bb::Encoder::Encoder(uint8_t pin_enc_a, uint8_t pin_enc_b, InputMode mode, Unit unit): 
  filtSpeed_(25, 100, true), 
  filtPos_(25, 100, true) {
  mode_ = mode;
  unit_ = unit;
  mmPT_ = 1.0;
  lastCycleUS_ = micros();
  presentPos_ = 0;
  pinEncA_ = pin_enc_a;
  pinEncB_ = pin_enc_b;

  pinMode(pinEncA_, INPUT);
  pinMode(pinEncB_, INPUT);

  for(enc_ = 0; encDescr_[enc_].taken == true; enc_++);
  if(enc_ >= NUM_ENC_SLOTS) return; // Should raise an exception here, but...

  switch(enc_) {
  case 0:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr0, CHANGE);
    break;
  case 1:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr1, CHANGE);
    break;
  case 2:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr2, CHANGE);
    break;
  case 3:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr3, CHANGE);
    break;
  case 4:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr4, CHANGE);
    break;
  case 5:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr5, CHANGE);
    break;
  case 6:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr6, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr6, CHANGE);
    break;
  case 7:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr7, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr7, CHANGE);
    break;
  case 8:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr8, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr8, CHANGE);
    break;
  case 9:
    attachInterrupt(digitalPinToInterrupt(pinEncA_), isr9, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinEncB_), isr9, CHANGE);
    break;
  default:
    bb::printf("Encoder ISR table full\n");
  }
  encDescr_[enc_].pinA = pin_enc_a;
  encDescr_[enc_].pinB = pin_enc_b;
  encDescr_[enc_].taken = true;
}

bb::Encoder::~Encoder() {
  if(enc_ < NUM_ENC_SLOTS) {
    detachInterrupt(pinEncA_);
    detachInterrupt(pinEncB_);
    encDescr_[enc_].taken = false;
  }
}

void bb::Encoder::setMode(Encoder::InputMode mode) {
  mode_ = mode;
}

void bb::Encoder::setUnit(Encoder::Unit unit) {
  unit_ = unit;
}

bb::Result bb::Encoder::update() {
  unsigned long ticks = encDescr_[enc_].value; // FIXME
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

void bb::Encoder::pinAChanged() {
  bb::printf("pin A changed\n");
  //encoderValue_++;
}

void bb::Encoder::pinBChanged() {
  bb::printf("pin B changed\n");
  //encoderValue_++;
}
