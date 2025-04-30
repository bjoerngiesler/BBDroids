#if !defined(DOCONFIG_H)
#define DOCONFIG_H

#include <Arduino.h>
#include <BBPacket.h>

// Basic Droid Config
static const bb::DroidType DROID_TYPE = bb::DroidType::DROID_DO;
static const char*         DROID_NAME = "Generic D-O";
static const uint8_t       BUILDER_ID = 0; // Reserved values: 0 Bjoern, 1 Felix, 2 Micke, 3 Brad, 4 Lars, 5 Lukas
static const uint8_t       DROID_ID   = 0;
static const float         WHEEL_CIRCUMFERENCE = 722.566;              // Wheel circumference in mm for D-O, required to convert between speed over ground to encoder ticks
static const float         WHEEL_TICKS_PER_TURN = 979.2 * (97.0/18.0); // 979.2 ticks per one turn of the drive gear, 18 teeth on the drive gear, 97 teeth on the main gear.
static const float         WHEEL_DISTANCE = 95.0;                      // Distance between the drive wheels
static const bool          HEAD_COUNTERWEIGHT = true;                  // set to false if you're running without a head counterweight

// Parameters - all of these can be set from the commandline and stored in flash.
struct DOParams {
    float neckRange         = 45.0; // Careful with this, easy to nosedive if the drive controller and neck aren't working together well.
    float neckOffset        = 0.0;
    float headRollRange     = 45.0;
    float headRollOffset    = 0.0;
    float headPitchRange    = 45.0; 
    float headPitchOffset   = 0.0;
    float headHeadingRange  = 90.0;
    float headHeadingOffset = 0.0;

    float motorDeadband     = 10.0;

    float wheelKp           = 0.06;
    float wheelKi           = 0.8;
    float wheelKd           = 0.0;

    float balKp             = 20;
    float balKi             = 0;
    float balKd             = 0;

    float autoPosKp             = 6;
    float autoPosKi             = 0.2;
    float autoPosKd             = 0;

    float posKp             = 1.2;
    float posKi             = 0.2;
    float posKd             = 0;

    float maxSpeed          = 1000;
    float accel             = 2500;

    float aerialOffset     = 0;
    float aerialAnim       = -45;

    float speedAxisGain     = 1.0;
    float rotAxisGain       = 0.4;

    float leanHeadToBody    = 0.0;

    // Free animation parameters. 
    // Be aware of the units!!! E.g. target unit for servo free animation is always degrees, but input may be something else.
    // Example - faNeckSpeed has input unit of mm/s, going up to maxSpeed, so is probably much below 1.
    float faNeckIMUAccel     = 0;     // Move neck by acceleration from the IMU
    float faNeckIMUPitch     = .8;
    float faNeckSPAccel      = -0.005; // Move neck by acceleration from speed setpoint
    float faNeckSpeed        = -0.01;  // Move neck by absolute speed over ground
    float faNeckSpeedSP      = -0.04;  // Move neck by speed *setpoint* over ground. 
    float faHeadPitchSpeedSP = -0.01;    // Pitch up head at higher speeds to counteract the neck speed setpoint
    float faHeadRollTurn     = .10;    // Move head roll by IMU turn speed
    float faHeadHeadingTurn  = .14;    // Move head heading by IMU turn speed
    float faAerialSpeedSP   = .05;     // Move aerials by speed over ground
    float faHeadAnnealTime   = .5;     // Time it takes for all head axes ot return to 0 after control has been relinquished.
    float faHeadAnnealDelay  = .3;     // Delay before we anneal the head position if button is released

    bool autoPosControl      = true;

    bb::HWAddress leftRemoteAddress = {0,0};
    bb::HWAddress rightRemoteAddress = {0,0};
};

// Battery constants
static const float POWER_BATT_NONE = 5.0;  // Everything under this means we're connected to USB.
static const float POWER_BATT_MIN  = 13.0; // Minimum voltage - below this, everything switches off to save the LiPos.
static const float POWER_BATT_MAX  = 16.0; // Maximum voltage - above this, we're overvolting and will probably breal stuff.

// Selftest Constants
static const float ST_MIN_PWM              = 40.0;
static const float ST_MAX_PWM              = 255.0;
static const float ST_ABORT_DISTANCE       = (2*WHEEL_DISTANCE*M_PI)/4.0;
static const float ST_MIN_DISTANCE         = ST_ABORT_DISTANCE / 8.0;     
static const float ST_ABORT_ACCEL          = 2.0;
static const float ST_MIN_ACCEL            = ST_ABORT_ACCEL / 3.0;
static const float ST_ABORT_MILLIAMPS      = 1800; // pure motor current - added to current measured at rest
static const float ST_ABORT_HEADING_CHANGE = 45.0;
static const float ST_MIN_HEADING_CHANGE   = ST_ABORT_HEADING_CHANGE / 4.0;

// Servo IDs
static const uint8_t SERVO_NECK         = 1;
static const uint8_t SERVO_HEAD_PITCH   = 2;
static const uint8_t SERVO_HEAD_HEADING = 3;
static const uint8_t SERVO_HEAD_ROLL    = 4;

// Pins
static const uint8_t PULL_DOWN_15      = 15; // A0 - Used as left motor driver GND
static const uint8_t P_LEFT_ENCB       = 16; // A1 - Must be an interrupt pin
static const uint8_t P_LEFT_ENCA       = 17; // A2 - Must be an interrupt pin
static const uint8_t P_LEFT_PWMA       = 18; // A3 - Must be a PWM pin
static const uint8_t P_LEFT_PWMB       = 19; // A4 - Must be a PWM pin
static const uint8_t PULL_DOWN_20      = 20; // A5 - Used as right motor driver GND
static const uint8_t P_DYNAMIXEL_RTS   = 21; // A6
static const uint8_t P_SERIALTX_TX     = 0;  // Usually the default. Must be usable as Serial TX
static const uint8_t P_SERIALTX_RX     = 1;  // Usually the default. Must be usable as Serial RX
static const uint8_t P_RIGHT_PWMB      = 2;  // Must be a PWM pin
static const uint8_t P_RIGHT_PWMA      = 3;  // Must be a PWM pin
static const uint8_t P_STATUS_NEOPIXEL = 4;
static const uint8_t P_BALL1_NEOPIXEL  = 5;

static const uint8_t P_RIGHT_ENCB      = 6;  // Must be an interrupt pin
static const uint8_t P_RIGHT_ENCA      = 7;  // Must be an interrupt pin
static const uint8_t P_DFPLAYER_TX     = 8;  // Must be usable as Serial TX
static const uint8_t P_DFPLAYER_RX     = 9;  // Must be usable as Serial RX
static const uint8_t P_BALL2_NEOPIXEL  = 10;
static const uint8_t P_I2C_SDA         = 11; // Usually the default
static const uint8_t P_I2C_SCL         = 12; // Usually the default
static const uint8_t P_DYNAMIXEL_RX    = 13; // Must be usable as Serial RX
static const uint8_t P_DYNAMIXEL_TX    = 14; // Must be usable as Serial TX

// I2C Peripherals
static const uint8_t BATT_STATUS_ADDR  = 0x40;
static const uint8_t IMU_ADDR          = 0x6a;
static const uint8_t AERIAL_ADDR       = 0x17;

#endif // DOCONFIG_H