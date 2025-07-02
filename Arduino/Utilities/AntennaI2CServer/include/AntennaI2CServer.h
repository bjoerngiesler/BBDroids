#if !defined(ANTENNAI2CSERVER_H)
#define ANTENNAI2CSERVER_H

#include <sys/types.h>
#include "Config.h"

namespace antenna {

// Eyes
// Bit 6-7: 00 - off, 01 - white, 10 - red, 11 - blue
// Bit 0-5: Size
static const uint8_t EYE_COLOR_SHIFT = 6;
static const uint8_t EYE_COLOR_MASK  = (3 << EYE_COLOR_SHIFT);
static const uint8_t EYE_SIZE_SHIFT  = 0;
static const uint8_t EYE_SIZE_MASK   = 0x1f;

static const uint8_t EYE_COLOR_OFF   = (0 << EYE_COLOR_SHIFT);
static const uint8_t EYE_COLOR_WHITE = (1 << EYE_COLOR_SHIFT);
static const uint8_t EYE_COLOR_RED   = (2 << EYE_COLOR_SHIFT);
static const uint8_t EYE_COLOR_BLUE  = (3 << EYE_COLOR_SHIFT);


#define MAKE_EYE_BYTE(color, size, pos) uint8_t(color | size<<EYE_SIZE_SHIFT | pos)

// Control
// Bit 5-7: 000 - off, 001 - selftest, 010 - OK, 011 - degraded, 100 - vel ctrl, 101 - auto pos ctrl, 110 - man pos ctrl, 111 - error
// Bit 3-4: Autoblink speed (0: off, 3: fastest)
// Bit 0-2: Brightness
static const uint8_t CONTROL_STRIP_SHIFT      = 5;
static const uint8_t CONTROL_STRIP_MASK       = (7 << CONTROL_STRIP_SHIFT);
static const uint8_t CONTROL_AUTOBLINK_SHIFT  = 3;
static const uint8_t CONTROL_AUTOBLINK_MASK   = (3 << CONTROL_AUTOBLINK_SHIFT);
static const uint8_t CONTROL_BRIGHTNESS_MASK  = 7;
static const uint8_t CONTROL_BRIGHTNESS_SHIFT = 0;

static const uint8_t CONTROL_STRIP_OFF       = (0 << CONTROL_STRIP_SHIFT); // switch off
static const uint8_t CONTROL_STRIP_SELFTEST  = (1 << CONTROL_STRIP_SHIFT); // bounce
static const uint8_t CONTROL_STRIP_OK        = (2 << CONTROL_STRIP_SHIFT); // slide
static const uint8_t CONTROL_STRIP_DEGRADED  = (3 << CONTROL_STRIP_SHIFT); // slide
static const uint8_t CONTROL_STRIP_VEL       = (4 << CONTROL_STRIP_SHIFT); // slide
static const uint8_t CONTROL_STRIP_AUTO_POS  = (5 << CONTROL_STRIP_SHIFT); // slide
static const uint8_t CONTROL_STRIP_MAN_POS   = (6 << CONTROL_STRIP_SHIFT); // slide
static const uint8_t CONTROL_STRIP_ERROR     = (7 << CONTROL_STRIP_SHIFT); // bounce


struct Parameters {
  uint8_t servoSetpoints[3] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
  uint8_t eyes[2] = {0, 0};
  uint8_t eyePos[2] = {0, 0};
  uint8_t control = CONTROL_STRIP_OFF;
};

};

#endif // ANTENNAI2CSERVER_H