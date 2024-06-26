#if !defined(BB8CONFIG_H)
#define BB8CONFIG_H

#include <Arduino.h>
#include <math.h>
#include <LibBB.h>

static const bb::DroidType DROID_TYPE = bb::DroidType::DROID_BB8;
static const char*         DROID_NAME = "Generic BB-8";
static const uint8_t       BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t       DROID_ID = 0;

// Network config
static const uint16_t COMMAND_UDP_PORT = 2000; // BB8 listens on this port for commands (see BB8Packet.h for command structure)
static const uint16_t STATE_UDP_PORT   = 2001; // BB8 sends running state on this port
static const uint16_t REPLY_UDP_PORT   = 2002; // This port is used to reply to special commands

#if 0
// Left side pins
static const uint8_t P_DRIVE_EN          = 15; // A0
static const uint8_t P_DRIVE_A           = 16; // A1
static const uint8_t P_DRIVE_B           = 17; // A2
static const uint8_t P_DRIVE_PWM         = 18; // A3
static const uint8_t P_YAW_PWM       = 19; // A4
static const uint8_t P_YAW_EN        = 20; // A5
static const uint8_t P_DYNAMIXEL_RTS   = 21; // A6
static const uint8_t P_SERIALTX_TX     = 0;  // OK
static const uint8_t P_SERIALTX_RX     = 1;  // OK
static const uint8_t P_YAW_A         = 2;
static const uint8_t P_YAW_B         = 3;
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
#endif

#if 1
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
#endif

//static const uint8_t DOME_IMU_ADDR     = 0x18;
//static const uint8_t BODY_IMU_ADDR     = 0x19;
static const uint8_t BATT1_STATUS_ADDR   = 0x40;
static const uint8_t BATT2_STATUS_ADDR   = 0x41;

static const uint8_t STATUSPIXEL_OVERALL = 0;
static const uint8_t STATUSPIXEL_REMOTE  = 1;
static const uint8_t STATUSPIXEL_MOTORS  = 2;

static const uint8_t BALL1_NEOPIXEL_COUNT = 10;
static const uint8_t BALL2_NEOPIXEL_COUNT = 10;

const long unsigned int DYNAMIXEL_BPS = 57600;

static const float DRIVE_SPEED_KP       = 0.065f;
static const float DRIVE_SPEED_KI       = 0.2f;
static const float DRIVE_SPEED_KD       = 0.0f;
static const float BAL_KP               = 25;
static const float BAL_KI               = 0;
static const float BAL_KD               = 0.2;
static const float ROLL_KP              = 0;
static const float ROLL_KI              = 2.0;
static const float ROLL_KD              = 0.25;
static const uint16_t ROLL_SERVO_KP     = 800;
static const uint16_t ROLL_SERVO_KI     = 10;
static const uint16_t ROLL_SERVO_KD     = 0;
static const float ROLL_SERVO_VEL       = 32000.0;

static const float DRIVE_SPEED_IMAX     = 10.0f;     // not used yet
static const float DRIVE_SPEED_DEADBAND = 0.01f; // not used yet
static const float DRIVE_SPEED_IABORT   = 1000.0f; // not used yet
static const float DRIVE_SPEED_MAX      = 400.0f;

static const float ROLL_IMAX            = 200;
static const float ROLL_TORQUE_PERCENT  = 90;

static const int DOME_HEADING_SERVO  = 1;
static const int DOME_ROLL_SERVO     = 2;
static const int DOME_PITCH_SERVO    = 3;
static const int BODY_ROLL_SERVO     = 4;
static const float DOME_MAX_VELOCITY = 130; // degrees per second

static const bool DOME_HEADING_SERVO_REVERSE = false;
static const bool DOME_ROLL_SERVO_REVERSE    = false;
static const bool DOME_PITCH_SERVO_REVERSE   = false;
static const bool BODY_ROLL_SERVO_REVERSE    = true;

// Motion Limits
static const float BODY_ROLL_RANGE     = 25.0;
static const float BODY_ROLL_OFFSET    = 0;
static const float DOME_PITCH_RANGE    = 30.0;
static const float DOME_PITCH_OFFSET   = 0.0;
static const float DOME_HEADING_RANGE  = 90.0;
static const float DOME_HEADING_OFFSET = 0.0;
static const float DOME_ROLL_RANGE     = 30.0;
static const float DOME_ROLL_OFFSET    = 0.0;

static const float BODY_CIRCUMFERENCE            = 2*M_PI*253.0; // bb8 motor pwm 0
static const float DRIVE_MOTOR_TICKS_PER_TURN    = 4776.384;
static const float DRIVE_MOTOR_MAX_SPEED_M_PER_S = 2.0;

static const uint8_t SOUND_FOLDER_SYSTEM  = 99;
static const uint8_t SOUND_STARTING_UP    = 1;
static const uint8_t SOUND_XBEE_OK        = 2;
static const uint8_t SOUND_XBEE_FAILURE   = 3;
static const uint8_t SOUND_WIFI_OK        = 4;
static const uint8_t SOUND_WIFI_FAILURE   = 5;
static const uint8_t SOUND_SERVOS_OK      = 6;
static const uint8_t SOUND_SERVOS_FAILURE = 7;
static const uint8_t SOUND_BB8_OK         = 8;
static const uint8_t SOUND_BB8_FAILURE    = 9;
static const uint8_t SOUND_MAINLOOP       = 255;

//#define BATT_VOLTAGE_PRECISE // if #defined, this causes BB8BattStatus::updateVoltage() compute the correct voltage from the bus and across the shunt, but it takes twice the time.

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

static const uint8_t IMU_ADDR           = 0x6a;


#endif // BB8CONFIG_H
