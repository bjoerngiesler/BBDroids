#if !defined(CONFIG_H)
#define CONFIG_H

#include <sys/types.h>

static const uint8_t I2C_ADDRESS  = 0x17;

static const uint8_t SERVO_MIN = 0;
static const uint8_t SERVO_MAX = 180;
static const uint8_t SERVO_CENTER = (SERVO_MAX-SERVO_MIN)/2 + SERVO_MIN;

static const uint8_t LED_PIN = D10;
static const uint8_t LED_COUNT = (3*17);

#endif // DEFINES_H