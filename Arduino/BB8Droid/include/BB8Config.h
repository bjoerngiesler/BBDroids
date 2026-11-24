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
static const float ROLL_REDUCTION                 = 1.0;  // servo degrees per degree of pendulum tilt. Servo now coupled 1:1 (old cable drive was 80/22).
static const float DOME_ROLL_REDUCTION            = 2.0;  // dome roll servo degrees per degree of dome motion around sphere center
static const float DOME_PITCH_REDUCTION           = 2.0;  // ASSUMPTION (same as roll) - dome pitch servo degrees per degree of dome motion around sphere center. Please measure.
static const float ROLL_RATE_FILTER_CUTOFF        = 15.0; // Hz, low-pass on gyro roll rate used for damping

struct BB8Params {
  // Motion Limits
  float bodyRollRange     = 25.0; // in the station don't go beyond 25, in the droid 30 is OK
  float bodyRollOffset    = 0;
  float bodyRollVel       = 500;
  float bodyRollAccel     = 60;    // was 30. 60 together with rollKdSpeed keeps the roll loop stable when cornering at speed (sim/README.md, 3.7)
  float domePitchRange    = 45.0;
  float domePitchOffset   = 0.0;
  float domePitchVel      = 100;
  float domeHeadingRange  = 90.0;
  float domeHeadingOffset = 0.0;
  float domeHeadingVel    = 100;
  float domeRollRange     = 20.0;
  float domeRollOffset    = 0.0;
  float domeRollVel       = 100;
 
  // PID controller parameters
  float driveSpeedKp       = 0.05f;
  float driveSpeedKi       = 0.2f;
  float driveSpeedKd       = 0.0f;

  float balKp              = 25;
  float balKi              = 0;
  float balKd              = 0.2;

  // Roll controller, all in body degrees (gear reduction applied at the output):
  // tilt = rollKff*goal + rollKp*err + rollKi*int(err) - rollKd*gyroRate
  // Note: old rollKp=2 was in servo degrees, equivalent to ~0.55 here.
  // Tuned in simulation for ROLL_REDUCTION=1, body_roll_accel=30, pendulum 4.5kg@155mm, head ~1kg (see sim/README.md).
  float rollKp             = 0.5;
  float rollKi             = 0.4;
  float rollKd             = 0.4;  // deg tilt per deg/s of measured roll rate
  float rollKdSpeed        = 1000; // mm/s, above this measured speed rollKd is scaled by rollKdSpeed/|speed| (0 = off)
  float rollKff            = 0.0;  // 1.0 = same open-loop lean as roll_direct. Off by default because
                                   // the right value depends on mass distribution - tune on hardware.

  float autoPosKp          = 0;
  float autoPosKi          = 0;
  float autoPosKd          = 0;

  float posKp             = 1.2;
  float posKi             = 0.2;
  float posKd             = 0;

  // Flags to modify roll behavior
  bool rollDirect          = false;
  bool rollInhibit         = false;
  bool rollDebug           = false;
  bool autoPosControl      = false;

  float driveSpeedDeadband = 0.01f;
  float driveAccelMax      = 1200;   // mm/s^2 ramp on the speed setpoint from the remote (0 = off). Also defines the commanded acceleration for the dome feedforward.
                                     // Was 2000, which is more than the drive delivers: the pendulum swings far forward and throws the head back (sim/README.md, 6.4).
  float driveSpeedIMax     = 255;    // Max I share of the drive speed controller in PWM: errI is bounded to +-driveSpeedIMax/(driveSpeedKi*encoder controlGain) (was a fixed errI bound of 255 = ~153 PWM incl. controlGain).
  float driveSpeedMax      = DRIVE_MOTOR_MAX_SPEED_MM_PER_S;
  float rollAngleMax       = 20.0;

  float rollIMax            = 50;
  // Faster return to upright: when the roll goal moves towards zero, bleed the I state towards the share
  // that belongs to the new goal with this time constant. Only active for rollReturnWindow seconds after
  // the stick moved towards center, so steady disturbances are still integrated away. 0 = off.
  float rollIBleedTau       = 0.4;   // s
  float rollReturnWindow    = 1.5;   // s
  // Optional input shaper on the roll goal (0 = off, 1 = ZV, 2 = ZVD), tuned to the rocking period.
  int   rollShaper          = 0;
  float rollShaperT         = 2.3;   // s, max 4 s at 100 Hz
  float rollTorquePercent   = 90;

  bool domeHeadingServoReverse = true;
  bool domeRollServoReverse    = true;
  bool domePitchServoReverse   = false;
  bool bodyRollServoReverse    = false;

  float faDomeAnnealTime     = .5;     // Time it takes for all head axes ot return to 0 after control has been relinquished.
  float faDomeAnnealDelay    = .3;     // Delay before we anneal the head position if button is released
  // Head compensation and feedforward: the "ideal" set from the simulation (sim/README.md, section 5/6), for
  // pendulum 4.5 kg @ 155 mm and an ESTIMATED head of 1 kg @ 320 mm. Signs are in the simulation's convention
  // and NOT verified on hardware - check them first (sim/README.md, section 8) and recompute after weighing the head.
  float faSpeedSPToDome      = .01;    // Dome pitch (servo deg) per mm/s of speed setpoint. Keeps the head upright at constant speed. Was -.01.
  float faRollSPToDome       = -.5;
  float faDomePitchToWeight  = -.85;   // Balance setpoint shift (deg pendulum pitch) per deg of measured dome pitch around sphere center.
                                       // Ideal -(m_head*L)/(m_weight*l - m_head*L). Was .5 but unused.
  float faAccelToDome        = 15;     // Dome pitch (deg around sphere center) per m/s^2 of commanded acceleration (pure feedforward, no IMU).
  float domeAccelTau         = 0.5;    // s, low-pass on commanded acceleration for faAccelToDome
  float faDomeSPAccel        = 0;      // Dome pitch (servo deg) per mm/s of speed error (ramped setpoint - measured speed), like D-O's faNeckSPAccel.
                                       // Uses the encoder, so it is feedback. 0 = off; sign in sim convention not verified (sim/README.md, 6.4).
  bool  domeCompRemoteOnly   = true;   // faDomePitchToWeight acts only on the remote share of the dome pitch, not on the automatic feedforward
  float faDomeRollToWeight   = -.46;   // Counterweight tilt per deg of measured dome roll, both around sphere center. Ideal -(m_head*L)/(m_weight*l). Was 0.
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
