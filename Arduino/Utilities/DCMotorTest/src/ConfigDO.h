#if !defined(CONFIG_H)
#define CONFIG_H

//#define DIRECTIONPIN_MOTOR_DRIVER
#define DUALPWM_MOTOR_DRIVER
static const uint8_t PIN_PWM_A_1   = 18;
static const uint8_t PIN_PWM_B_1   = 19;
static const uint8_t PIN_PWM_A_2   = 3;
static const uint8_t PIN_PWM_B_2   = 2;

static const uint8_t PIN_ENC_A_1 = 17;
static const uint8_t PIN_ENC_B_1 = 16;
static const uint8_t PIN_ENC_A_2 = 7;
static const uint8_t PIN_ENC_B_2 = 6;

DCMotor motor[2] = { DCMotor(PIN_PWM_A_1, PIN_PWM_B_1), DCMotor(PIN_PWM_A_2, PIN_PWM_B_2) };
bb::Encoder input[2] = {bb::Encoder(PIN_ENC_A_1, PIN_ENC_B_1), bb::Encoder(PIN_ENC_A_2, PIN_ENC_B_2)};

// Values for D-O. This gives about 0.14mm per tick.
static const float WHEEL_CIRCUMFERENCE = 722.566310325652445;
static const float WHEEL_TICKS_PER_TURN = 979.2 * (97.0/18.0); // 979 ticks per one turn of the drive gear, 18 teeth on the drive gear, 96 teeth on the main gear.
float speedKp = 0.13, speedKi = 0.8, speedKd = 0.0;
float speedCutoff = 25;
float posKp = 0.05, posKi = 0.0, posKd = 0.0;
float posCutoff = 25;

bool motor1Reverse = false, motor2Reverse = false;

#else // CONFIG_H
#error Two Config headers included, this is likely not what you want!
#endif