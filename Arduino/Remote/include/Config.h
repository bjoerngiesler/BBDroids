#if !defined(CONFIG_H)
#define CONFIG_H

#include <Arduino.h>

#if defined(VERSION)
#define STRINGIFY(s) #s
#define XSTRINGIFY(s) STRINGIFY(s)
#define VERSION_STRING "V" XSTRINGIFY(VERSION)
#else
#error VERSION not defined
#endif

#if defined(ARDUINO_ARCH_ESP32)

struct Pins {
    uint8_t P_D_NEOPIXEL;
    uint8_t P_A_JOY_VER, P_A_JOY_HOR; 
    uint8_t P_A_BATT_CHECK;
    uint8_t P_D_XBEE_RX, P_D_XBEE_TX;
    uint8_t P_D_DISPLAY_RX, P_D_DISPLAY_TX;
    uint8_t P_A_POT1, P_A_POT2;
    uint8_t P_D_RST_IOEXP;
};

static const Pins leftRemotePins = {
    D0,         // P_D_NEOPIXEL
    A1, A2,     // P_A_JOY_VER, P_A_JOY_HOR
    A3,         // P_A_BATT_CHECK
    D7, D6,     // P_D_XBEE_RX, P_D_XBEE_TX
    D8, D9,     // P_D_DISPLAY_RX, P_D_DISPLAY_TX
    A10, 0xff,  // P_A_POT1, UNDEF(P_A_POT2),
    0xff        // UNDEF(P_D_RST_IOEXP)
};

static const Pins rightRemotePins = {
    D9,         // P_D_NEOPIXEL
    A8, A3,     // P_A_JOY_VER, P_A_JOY_HOR,
    A10,        // P_A_BATT_CHECK
    D7, D6,     // P_D_XBEE_RX, P_D_XBEE_TX
    0xff, 0xff, // UNDEF(P_D_DISPLAY_RX, P_D_DISPLAY_TX)
    A0, A1,     // P_A_POT1, P_A_POT2,
    D2          // P_D_RST_IOEXP
};

extern Pins pins;
extern bool isLeftRemote;

/*
// LEFT_REMOTE
static const uint8_t P_D_NEOPIXEL   = D0;
static const uint8_t P_A_JOY_VER    = A1;
static const uint8_t P_A_JOY_HOR    = A2;
static const uint8_t P_A_BATT_CHECK = A3;
static const uint8_t P_D_XBEE_TX    = D6;
static const uint8_t P_D_XBEE_RX    = D7;
static const uint8_t P_D_DISPLAY_RX = D8;
static const uint8_t P_D_DISPLAY_TX = D9;
static const uint8_t P_A_POT1       = A10;
 
// RIGHT REMOTE
static const uint8_t P_A_POT1       = A0;
static const uint8_t P_A_POT2       = A1;
static const uint8_t P_D_RST_IOEXP  = D2;
static const uint8_t P_A_JOY_HOR    = A3;
static const uint8_t P_D_XBEE_TX    = D6;
static const uint8_t P_D_XBEE_RX    = D7;
static const uint8_t P_A_JOY_VER    = A8;
static const uint8_t P_D_NEOPIXEL   = D9;
static const uint8_t P_A_BATT_CHECK = A10;
*/

static const uint8_t P_DETECT_WHICH_REMOTE = A2;

#else // !defined(ARDUINO_ARCH_ESP32)

#if defined(LEFT_REMOTE)

static const uint8_t P_A_ENC         = A0;
static const uint8_t P_A_BATT_CHECK  = A5;
static const uint8_t P_A_JOY_VER     = A2;
static const uint8_t P_A_JOY_HOR     = A3; 

static const uint8_t P_D_BTN_PINKY   = D11;
static const uint8_t P_D_BTN_INDEX   = D9;
static const uint8_t P_D_BTN_JOY     = D5;
static const uint8_t P_D_BTN_TOP_L   = D7;
static const uint8_t P_D_BTN_TOP_R   = D10;
static const uint8_t P_D_BTN_CONFIRM = D8;
static const uint8_t P_D_NEOPIXEL    = D4;
static const uint8_t P_D_BTN_L       = D2;
static const uint8_t P_D_BTN_R       = D3;

static const uint8_t P_NEOPIXEL      = D4;
static const uint8_t P_DISPLAY_RX    = D15;
static const uint8_t P_DISPLAY_TX    = D6;
static const uint8_t P_DISPLAY_RESET = D10;

#else // defined(LEFT_REMOTE)

static const uint8_t P_A_POT1        = A0;
static const uint8_t P_A_POT2        = A1;
static const uint8_t P_A_BATT_CHECK  = A5;
static const uint8_t P_A_JOY_VER     = A2;
static const uint8_t P_A_JOY_HOR     = A3;

static const uint8_t P_D_BTN_PINKY   = D10;
static const uint8_t P_D_BTN_INDEX   = D9;
static const uint8_t P_D_BTN_JOY     = D8;
static const uint8_t P_D_BTN_TOP_L   = D7;
static const uint8_t P_D_BTN_TOP_R   = D6;
static const uint8_t P_D_BTN_CONFIRM = D5;
static const uint8_t P_D_NEOPIXEL    = D4;
static const uint8_t P_D_BTN_L       = D3;
static const uint8_t P_D_BTN_R       = D2;

#endif

#endif // ARDUINO_ARCH_ESP32

static const uint8_t BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t REMOTE_ID = 0;

// Fixme - can use these to distinguish
static const uint8_t LEFT_IMU_ADDR       = 0x6b;
static const uint8_t RIGHT_IMU_ADDR      = 0x6b;

static const uint8_t MCP_ADDR1           = 0x27;
static const uint8_t MCP_ADDR2           = 0x26;

//static float MAX_VOLTAGE = 4.2;
//static float MIN_VOLTAGE = 3.7;
static const uint16_t MAX_ANALOG_IN_VDIV = 2520; // Max battery voltage is 4.2V, with our voltage divider we'll see 2.1V or 2606 out of 4096 possible values.
static const uint16_t MIN_ANALOG_IN_VDIV = 1940; // Min battery voltage is 3.7V, with our voltage divider we'll see 1.85V or 2296 out of 4096 possible values.

#endif // CONFIG_H