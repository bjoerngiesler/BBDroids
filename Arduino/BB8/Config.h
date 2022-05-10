#include <Arduino.h>

// network config
#define WIFI_SSID   "BB8Server"
#define WIFI_WPA_KEY "BB8Server"
#define REMOTE_UDP_PORT 2000

// motor ports
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

#define DEADBAND_MIN              510
#define DEADBAND_MAX              512
