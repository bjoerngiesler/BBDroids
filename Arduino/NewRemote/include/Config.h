#if !defined(CONFIG_H)
#define CONFIG_H

#include <Arduino.h>
#include <vector>

#if defined(VERSION)
#define STRINGIFY(s) #s
#define XSTRINGIFY(s) STRINGIFY(s)
#define VERSION_STRING "V" XSTRINGIFY(VERSION)
#else
#error VERSION not defined
#endif

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

static const uint8_t P_DETECT_WHICH_REMOTE = A2;

static const uint8_t BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t REMOTE_ID = 0;

// Fixme - can use these to distinguish
static const std::vector<uint8_t> IMU_ADDRESSES = {0x6a, 0x6b, 0x6c};

static const uint8_t MCP_ADDR1           = 0x27;
static const uint8_t MCP_ADDR2           = 0x26;

//static float MAX_VOLTAGE = 4.2;
//static float MIN_VOLTAGE = 3.7;
static const uint16_t MAX_ANALOG_IN_VDIV = 2520; // Max battery voltage is 4.2V, with our voltage divider we'll see 2.1V or 2606 out of 4096 possible values.
static const uint16_t MIN_ANALOG_IN_VDIV = 1940; // Min battery voltage is 3.7V, with our voltage divider we'll see 1.85V or 2296 out of 4096 possible values.

#endif // CONFIG_H