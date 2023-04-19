#if !defined(BB8CONFIG_H)
#define BB8CONFIG_H

#include <Arduino.h>
#include <math.h>

// Network config
static const bool  WIFI_AP_MODE  = true;
static const String WIFI_SSID    = "BB8Server";
static const String WIFI_WPA_KEY = "BB8ServerKey";

static const uint16_t COMMAND_UDP_PORT = 2000; // BB8 listens on this port for commands (see BB8Packet.h for command structure)
static const uint16_t STATE_UDP_PORT   = 2001; // BB8 sends running state on this port
static const uint16_t REPLY_UDP_PORT   = 2002; // This port is used to reply to special commands

// Left side pins
static const uint8_t P_YAW_EN          = 15; // A0
static const uint8_t P_YAW_A           = 16; // A1
static const uint8_t P_YAW_B           = 17; // A2
static const uint8_t P_YAW_PWM         = 18; // A3
static const uint8_t P_DRIVE_PWM       = 19; // A4
static const uint8_t P_DRIVE_EN        = 20; // A5
static const uint8_t P_DYNAMIXEL_RTS   = 21; // A6
static const uint8_t P_SERIALTX_TX     = 0;  // OK
static const uint8_t P_SERIALTX_RX     = 1;  // OK
static const uint8_t P_DRIVE_A         = 2;
static const uint8_t P_DRIVE_B         = 3;
static const uint8_t P_STATUS_NEOPIXEL = 4;
static const uint8_t P_BALL1_NEOPIXEL  = 5;

// Right side pins
static const uint8_t P_DRIVEENC_B      = 6;   // OK
static const uint8_t P_DRIVEENC_A      = 7;   // OK
static const uint8_t P_DFPLAYER_TX     = 8;   // OK
static const uint8_t P_DFPLAYER_RX     = 9;   // OK
static const uint8_t P_BALL2_NEOPIXEL  = 10;
static const uint8_t P_I2C_SDA         = 11;  // OK
static const uint8_t P_I2C_SCL         = 12;  // OK
static const uint8_t P_DYNAMIXEL_RX    = 13;
static const uint8_t P_DYNAMIXEL_TX    = 14;

//static const uint8_t DOME_IMU_ADDR     = 0x18;
//static const uint8_t BODY_IMU_ADDR     = 0x19;

static const uint16_t DEADBAND         = 0.01f;

static const uint8_t STATUSPIXEL_OVERALL = 0;
static const uint8_t STATUSPIXEL_NETWORK = 1;
static const uint8_t STATUSPIXEL_MOTORS  = 2;

const long unsigned int DYNAMIXEL_BPS = 57600;

static const float ROLL_CONTROL_KP = 0.2f;
static const float ROLL_CONTROL_KI = 0.0f;
static const float ROLL_CONTROL_KD = 0.0f;
static const float ROLL_CONTROL_IMAX = 10.0f;
static const float ROLL_CONTROL_DEADBAND = 0.01f;
static const float ROLL_CONTROL_IABORT = 1000.0f;

static const int DOME_HEADING_SERVO = 1;
static const int DOME_ROLL_SERVO    = 2;
static const int DOME_PITCH_SERVO   = 3;
static const int BODY_ROLL_SERVO    = 4;

static const float BODY_CIRCUMFERENCE            = 2*M_PI*253.0; // bb8 motor pwm 0
static const float DRIVE_MOTOR_TICKS_PER_TURN    = 4776.384;
static const float DRIVE_MOTOR_MAX_SPEED_M_PER_S = 2.0;

//#define SERIALTX_MODE_SPEKTRUM
#define SERIALTX_MODE_XBEE

typedef struct {
  float min, max, offset, speed;
} ServoLimits;

extern ServoLimits servolimits[];

typedef enum {
  PLOT_NONE,
  PLOT_ROLL_CONTROLLER,
  PLOT_BODY_IMU
} PlotMode;

extern PlotMode plotMode;

#endif // BB8CONFIG_H
