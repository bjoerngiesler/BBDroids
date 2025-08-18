#if !defined(BB8CONFIG_H)
#define BB8CONFIG_H

#include <Arduino.h>
#include <math.h>
#include <LibBB.h>

static const bb::DroidType DROID_TYPE = bb::DroidType::DROID_BB8;
static const char*         DROID_NAME = "Generic BB-8";
static const uint8_t       BUILDER_ID = 0; // Reserved values: 0 - Bjoern, 1 - Felix, 2 - Micke, 3 - Brad
static const uint8_t       DROID_ID = 0;

static const float BODY_CIRCUMFERENCE             = 2*M_PI*253.0;
static const float DRIVE_MOTOR_TICKS_PER_TURN     = 4776.384;
static const float DRIVE_MOTOR_MAX_SPEED_MM_PER_S = 2000.0;
static const float ROLL_REDUCTION                 = 80.0/22.0;

struct BB8Params {
  // Motion Limits
  float bodyRollRange     = 25.0; // in the station don't go beyond 25, in the droid 30 is OK
  float bodyRollOffset    = 2;
  float domePitchRange    = 45.0;
  float domePitchOffset   = 0.0;
  float domeHeadingRange  = 90.0;
  float domeHeadingOffset = 0.0;
  float domeRollRange     = 45.0;
  float domeRollOffset    = 0.0;

  // PID controller parameters
  float driveSpeedKp       = 0.065f;
  float driveSpeedKi       = 0.2f;
  float driveSpeedKd       = 0.0f;
  float balKp              = 25;
  float balKi              = 0;
  float balKd              = 0.2;
  float rollKp             = 1.05;
  float rollKi             = 0;
  float rollKd             = 0;
  float rollServoVel       = 0;
  float rollServoAccel     = 0;

  bool rollDirect          = false;
  bool rollInhibit         = false;
  bool rollDebug           = false;
  bool autoPosControl      = false;

  float driveSpeedDeadband = 0.01f; // not used yet
  float driveSpeedMax      = DRIVE_MOTOR_MAX_SPEED_MM_PER_S;
  float rollAngleMax       = 20.0;

  float rollIMax            = 50;
  float rollTorquePercent   = 90;
  float domeMaxVel          = 130; // degrees per second

  bool domeHeadingServoReverse = true;
  bool domeRollServoReverse    = true;
  bool domePitchServoReverse   = false;
  bool bodyRollServoReverse    = true;

  float faDomeAnnealTime   = .5;     // Time it takes for all head axes ot return to 0 after control has been relinquished.
  float faDomeAnnealDelay  = .3;     // Delay before we anneal the head position if button is released
};

static const int DOME_HEADING_SERVO  = 1;
static const int DOME_ROLL_SERVO     = 2;
static const int DOME_PITCH_SERVO    = 3;
static const int BODY_ROLL_SERVO     = 4;

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

static const uint8_t BATT1_STATUS_ADDR   = 0x40;
static const uint8_t BATT2_STATUS_ADDR   = 0x41;

static const uint8_t BALL1_NEOPIXEL_COUNT = 10;
static const uint8_t BALL2_NEOPIXEL_COUNT = 10;

// I2C Peripherals
static const uint8_t IMU_ADDR           = 0x6a;


#endif // BB8CONFIG_H
