#if !defined(DOCONFIG_H)
#define DOCONFIG_H

#include <Arduino.h>
#include <BBPacket.h>

static const bb::DroidType DROID_TYPE = bb::DroidType::DROID_DO;
static const char*         DROID_NAME = "Generic D-O";

// Network config
static const uint16_t COMMAND_UDP_PORT = 2000; // BB8 listens on this port for commands (see BB8Packet.h for command structure)
static const uint16_t STATE_UDP_PORT   = 2001; // BB8 sends running state on this port
static const uint16_t REPLY_UDP_PORT   = 2002; // This port is used to reply to special commands

#if 0
// Left side pins
static const uint8_t PULL_DOWN_15      = 15; // A0 // Used as left motor driver GND
static const uint8_t P_LEFT_ENCB       = 16; // A1 
static const uint8_t P_LEFT_ENCA       = 17; // A2 
static const uint8_t P_LEFT_PWMA       = 18; // A3 
static const uint8_t P_LEFT_PWMB       = 19; // A4 
static const uint8_t PULL_DOWN_20      = 20; // A5 // Used as right motor driver GND
static const uint8_t P_DYNAMIXEL_RTS   = 21; // A6
static const uint8_t P_SERIALTX_TX     = 0;  // OK
static const uint8_t P_SERIALTX_RX     = 1;  // OK
static const uint8_t P_RIGHT_PWMB      = 2;  // OK
static const uint8_t P_RIGHT_PWMA      = 3;  // OK
static const uint8_t P_STATUS_NEOPIXEL = 4;
static const uint8_t P_BALL1_NEOPIXEL  = 5;

// Right side pins
static const uint8_t P_RIGHT_ENCB      = 6;   // OK
static const uint8_t P_RIGHT_ENCA      = 7;   // OK
static const uint8_t P_DFPLAYER_TX     = 8;   // OK
static const uint8_t P_DFPLAYER_RX     = 9;   // OK
static const uint8_t P_BALL2_NEOPIXEL  = 10;
static const uint8_t P_I2C_SDA         = 11;  // OK
static const uint8_t P_I2C_SCL         = 12;  // OK
static const uint8_t P_DYNAMIXEL_RX    = 13;
static const uint8_t P_DYNAMIXEL_TX    = 14;
#else
// Left side pins
static const uint8_t PULL_DOWN_15      = 15; // A0 // Used as left motor driver GND
static const uint8_t P_LEFT_ENCB       = 16; // A1 
static const uint8_t P_LEFT_ENCA       = 17; // A2 
static const uint8_t P_LEFT_PWMA       = 18; // A3 
static const uint8_t P_LEFT_PWMB       = 19; // A4 
static const uint8_t PULL_DOWN_20      = 20; // A5 // Used as right motor driver GND
static const uint8_t P_DYNAMIXEL_RTS   = 21; // A6
static const uint8_t P_SERIALTX_TX     = 0;  // OK
static const uint8_t P_SERIALTX_RX     = 1;  // OK
static const uint8_t P_RIGHT_PWMB      = 2;  // OK
static const uint8_t P_RIGHT_PWMA      = 3;  // OK
static const uint8_t P_STATUS_NEOPIXEL = 4;
static const uint8_t P_BALL1_NEOPIXEL  = 5;

// Right side pins
static const uint8_t P_RIGHT_ENCB      = 6;   // OK
static const uint8_t P_RIGHT_ENCA      = 7;   // OK
static const uint8_t P_DFPLAYER_TX     = 8;   // OK
static const uint8_t P_DFPLAYER_RX     = 9;   // OK
static const uint8_t P_BALL2_NEOPIXEL  = 10;
static const uint8_t P_I2C_SDA         = 11;  // OK
static const uint8_t P_I2C_SCL         = 12;  // OK
static const uint8_t P_DYNAMIXEL_RX    = 13;
static const uint8_t P_DYNAMIXEL_TX    = 14;
#endif

static const float WHEEL_CIRCUMFERENCE = 722.566310325652445;
static const float WHEEL_TICKS_PER_TURN = 979.2 * (97.0/18.0); // 979 ticks per one turn of the drive gear, 18 teeth on the drive gear, 97 teeth on the main gear.
static const float WHEEL_DISTANCE = 95.0;

static const float ST_MIN_PWM = 40.0;
static const float ST_MAX_PWM = 255.0;
static const float ST_ABORT_DISTANCE       = (2*WHEEL_DISTANCE*M_PI)/4.0;
static const float ST_MIN_DISTANCE         = ST_ABORT_DISTANCE / 8.0;     
static const float ST_ABORT_ACCEL          = 2.0;
static const float ST_MIN_ACCEL            = ST_ABORT_ACCEL / 3.0;
static const float ST_ABORT_MILLIAMPS      = 1800;
static const float ST_ABORT_HEADING_CHANGE = 45.0;
static const float ST_MIN_HEADING_CHANGE   = ST_ABORT_HEADING_CHANGE / 4.0;

static const uint8_t BATT_STATUS_ADDR   = 0x40;
static const uint8_t IMU_ADDR           = 0x6a;

static const float SPEED_KP = 0.013;
static const float SPEED_KI = 1.0; //0.8; 
static const float SPEED_KD = 0;
static const float POS_KP = 0.05;
static const float POS_KI = 0;
static const float POS_KD = 0;
static const float BAL_KP = 40;
static const float BAL_KI = 0;
static const float BAL_KD = 4;
static const float PITCH_BIAS = 1.35;
static const float PITCH_DEADBAND = .5;
static const float SPEED_REMOTE_FACTOR = 30.0;
static const float ROT_REMOTE_FACTOR = 250.0;


static const uint8_t SERVO_NECK         = 1;
static const uint8_t SERVO_HEAD_PITCH   = 2;
static const uint8_t SERVO_HEAD_HEADING = 3;
static const uint8_t SERVO_HEAD_ROLL    = 4;

static const float NECK_RANGE         = 20.0;
static const float HEAD_PITCH_RANGE   = 20.0;
static const float HEAD_HEADING_RANGE = 20.0;
static const float HEAD_ROLL_RANGE    = 20.0;

static const float GYRO_PITCH_DEADBAND = 1.0;

static const float POWER_BATT_NONE = 5.0;  // Everything under this means we're connected to USB.
static const float POWER_BATT_MIN  = 14.2; // Minimum voltage - below this, everything switches off to save the LiPos.
static const float POWER_BATT_MAX  = 16.0; // Maximum voltage - above this, we're overvolting and will probably breal stuff.

static const uint8_t BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t DROID_ID = 0;

#endif // DOCONFIG_H