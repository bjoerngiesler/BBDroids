#if !defined(CONFIG_H)
#define CONFIG_H

#include <Arduino.h>

// network config
#define WIFI_SSID   "BB8Server"
#define WIFI_WPA_KEY "BB8Server"
#define REMOTE_UDP_PORT 2000

#if 0
#define PIO_RX_TX_DYNAMIXEL        A0
#define PO_B_YAW                   A1
#define PO_A_YAW                   A2
#define PO_PWM_YAW                 A3
#define PO_EN_YAW                  A4
#define P_A5_UNUSED                A5
#define PO_EN_DRIVE                A6
#define PO_PWM_DRIVE               0
#define PO_B_DRIVE                 1
#define PO_A_DRIVE                 2
#define P_D3_UNUSED                3
#define PO_STATUS_NEOPIXEL         4
#define PO_BALL_NEOPIXEL           5
#define PI_B_DRIVEENC              6
#define PI_A_DRIVEENC              7
#define PO_RX_DFPLAYER             8
#define PO_TX_DFPLAYER             9
#define P_D10_UNUSED               10
#define P_D11_UNUSED               11
#define PO_DIR_I2C                 12
#define PI_DIN_I2C                 13
#define PO_DOUT_I2C                14
#endif

static const uint8_t P_MOT_EN          = 15;
static const uint8_t P_YAW_A           = 16;
static const uint8_t P_YAW_B           = 17;
static const uint8_t P_YAW_PWM         = 18;
static const uint8_t P_DRIVE_PWM       = 19;
static const uint8_t P_DRIVE_A         = 20;
static const uint8_t P_DRIVE_B         = 21;
static const uint8_t P_DYNAMIXEL_TX    = 0;
static const uint8_t P_DYNAMIXEL_RX    = 1;
static const uint8_t P_DYNAMIXEL_RTS   = 2;
static const uint8_t P_UNUSED_3        = 3;
static const uint8_t P_STATUS_NEOPIXEL = 4;
static const uint8_t P_BALL_NEOPIXEL   = 5;
static const uint8_t P_B_DRIVEENC      = 6;
static const uint8_t P_A_DRIVEENC      = 7;
static const uint8_t P_DFPLAYER_TX     = 8;
static const uint8_t P_DFPLAYER_RX     = 9;
static const uint8_t P_UNUSED_10       = 10;
static const uint8_t P_I2C_SDA         = 11;
static const uint8_t P_I2C_SCL         = 12;
static const uint8_t P_RX_XBEE         = 13;
static const uint8_t P_TX_XBEE         = 14;

static const uint16_t DEADBAND_MIN     = 510;
static const uint16_t DEADBAND_MAX     = 512;

#endif // CONFIG_H
