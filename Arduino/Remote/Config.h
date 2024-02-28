#if !defined(CONFIG_H)
#define CONFIG_H

#include <Arduino.h>

//#define LEFT_REMOTE 

#if defined(LEFT_REMOTE)

#define P_A_ENC         A0
#define P_A_BATT_CHECK  A5
#define P_A_JOY_VER     A2
#define P_A_JOY_HOR     A3 

#define P_D_BTN_PINKY   D11
#define P_D_BTN_INDEX   D9
#define P_D_BTN_JOY     D5
#define P_D_BTN_TOP_L   D7
#define P_D_BTN_TOP_R   D10
#define P_D_BTN_CONFIRM D8
#define P_D_NEOPIXEL    D4
#define P_D_BTN_L       D2
#define P_D_BTN_R       D3

#define P_NEOPIXEL      D4
#define P_DISPLAY_RX    D15
#define P_DISPLAY_TX    D6
#define P_DISPLAY_RESET D10

#else // defined(LEFT_REMOTE)

#define P_A_POT1        A0
#define P_A_POT2        A1
#define P_A_BATT_CHECK  A5
#define P_A_JOY_VER     A2
#define P_A_JOY_HOR     A3

#define P_D_BTN_PINKY   D10
#define P_D_BTN_INDEX   D9
#define P_D_BTN_JOY     D8
#define P_D_BTN_TOP_L   D7
#define P_D_BTN_TOP_R   D6
#define P_D_BTN_CONFIRM D5
#define P_D_NEOPIXEL    D4
#define P_D_BTN_L       D3
#define P_D_BTN_R       D2

#define P_NEOPIXEL      D4

#endif

#define JoystickEpsilon             0.01f

static const uint8_t BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t REMOTE_ID = 0;

#endif // CONFIG_H