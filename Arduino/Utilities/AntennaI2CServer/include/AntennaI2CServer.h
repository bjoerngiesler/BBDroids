#if !defined(ANTENNAI2CSERVER_H)
#define ANTENNAI2CSERVER_H

#include <sys/types.h>
#include "Config.h"

namespace antenna {

// Eyes
// Bit 6-7: 00 - off, 01 - white, 10 - red, 11 - blue
// Bit 3-5: Size
// Bit 0-2: Position

static const uint8_t EYE_COLOR_OFF   = (0 << 6);
static const uint8_t EYE_COLOR_WHITE = (1 << 6);
static const uint8_t EYE_COLOR_RED   = (2 << 6);
static const uint8_t EYE_COLOR_BLUE  = (3 << 6);

#define MAKE_EYE_BYTE(color, size, pos) uint8_t(color | size<<3 | pos)

// Control
// Bit 5-7: 000 - off, 001 - selftest, 010 - OK, 011 - degraded, 100 - vel ctrl, 101 - auto pos ctrl, 110 - man pos ctrl, 111 - error
// Bit 4: reserved
// Bit 3: 0 - autoblink off, 1 - autoblink on
// Bit 0-2: Brightness
static const uint8_t CONTROL_STRIP_OFF      = (0 << 5); // switch off
static const uint8_t CONTROL_STRIP_SELFTEST = (1 << 5); // bounce
static const uint8_t CONTROL_STRIP_OK       = (2 << 5); // slide
static const uint8_t CONTROL_STRIP_DEGRADED = (3 << 5); // slide
static const uint8_t CONTROL_STRIP_VEL      = (4 << 5); // slide
static const uint8_t CONTROL_STRIP_AUTO_POS = (5 << 5); // slide
static const uint8_t CONTROL_STRIP_MAN_POS  = (6 << 5); // slide
static const uint8_t CONTROL_STRIP_ERROR    = (7 << 5); // bounce

static const uint8_t CONTROL_STRIP_MASK     = (7 << 5);

struct Parameters {
  uint8_t servoSetpoints[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
  uint8_t eyes[2] = {0, 0};
  uint8_t control = CONTROL_STRIP_OFF;
};

};

#endif // ANTENNAI2CSERVER_H