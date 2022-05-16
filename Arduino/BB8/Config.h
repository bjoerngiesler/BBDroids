#if !defined(CONFIG_H)
#define CONFIG_H

#include <Arduino.h>

// network config
#define WIFI_SSID   "BB8Server"
#define WIFI_WPA_KEY "BB8Server"
#define REMOTE_UDP_PORT 2000

static const uint8_t P_MOT_EN          = 15; // A0
static const uint8_t P_YAW_A           = 16; // A1
static const uint8_t P_YAW_B           = 17; // A2
static const uint8_t P_YAW_PWM         = 18; // A3
static const uint8_t P_DRIVE_PWM       = 19; // A4
static const uint8_t P_UNUSED_20       = 20; // A5
static const uint8_t P_DYNAMIXEL_RTS   = 21; // A6
static const uint8_t P_SERIALTX_TX     = 0;
static const uint8_t P_SERIALTX_RX     = 1;
static const uint8_t P_DRIVE_A         = 2;
static const uint8_t P_DRIVE_B         = 3;
static const uint8_t P_STATUS_NEOPIXEL = 4;
static const uint8_t P_BALL_NEOPIXEL   = 5;
static const uint8_t P_DRIVEENC_B      = 6;   // OK
static const uint8_t P_DRIVEENC_A      = 7;   // OK
static const uint8_t P_DFPLAYER_TX     = 8;   // OK
static const uint8_t P_DFPLAYER_RX     = 9;   // OK
static const uint8_t P_UNUSED_10       = 10;
static const uint8_t P_I2C_SDA         = 11;  // OK
static const uint8_t P_I2C_SCL         = 12;  // OK
static const uint8_t P_DYNAMIXEL_RX    = 13;
static const uint8_t P_DYNAMIXEL_TX    = 14;

static const uint8_t DOME_IMU_ADDR     = 0x18;
static const uint8_t BODY_IMU_ADDR     = 0x19;

static const uint16_t DEADBAND_MIN     = 510;
static const uint16_t DEADBAND_MAX     = 512;

const long unsigned int DYNAMIXEL_BPS = 1000000;

#define SERIALTX_MODE_SPEKTRUM
//#define SERIALTX_MODE_XBEE

#endif // CONFIG_H
