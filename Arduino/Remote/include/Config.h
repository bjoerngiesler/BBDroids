#if !defined(CONFIG_H)
#define CONFIG_H

#include <Arduino.h>

#define ESP32_REMOTE

#define LEFT_REMOTE

#if defined(ESP32_REMOTE)

// The ESP32 remote uses a different pinout, and much less pins because
// it employs a MPC23017 i2c I/O expander for the buttons.

#if defined(LEFT_REMOTE)

static const uint8_t P_D_NEOPIXEL   = D0;
static const uint8_t P_A_JOY_VER    = A1;
static const uint8_t P_A_JOY_HOR    = A2;
static const uint8_t P_A_BATT_CHECK = A3;
static const uint8_t P_D_XBEE_TX    = D6;
static const uint8_t P_D_XBEE_RX    = D7;
static const uint8_t P_D_DISPLAY_TX = D8;
static const uint8_t P_D_DISPLAY_RX = D9;
static const uint8_t P_A_POT1       = A10;
 
 
#else // !defined(LEFT_REMOTE)

static const uint8_t P_A_POT2       = A0;
static const uint8_t P_A_POT1       = A1;
static const uint8_t P_D_RST_IOEXP  = D2;
static const uint8_t P_A_JOY_HOR    = A3;
static const uint8_t P_D_XBEE_TX    = D6;
static const uint8_t P_D_XBEE_RX    = D7;
static const uint8_t P_A_JOY_VER    = A8;
static const uint8_t P_D_NEOPIXEL   = D9;
static const uint8_t P_A_BATT_CHECK = A10;

#endif

#else // !defined(ESP32_REMOTE)

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

#endif // ESP32_REMOTE

#define JoystickEpsilon             0.01f

static const uint8_t BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t REMOTE_ID = 0;

static const uint8_t IMU_ADDR           = 0x6b;

#endif // CONFIG_H