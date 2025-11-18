#if !defined(CONFIG_H)
#define CONFIG_H

static const uint8_t PIN_PWM_M1_A  = 9;
static const uint8_t PIN_PWM_M1_B  = 8;
static const uint8_t PIN_PWM_M2_A  = 11;
static const uint8_t PIN_PWM_M2_B  = 10;
static const uint8_t PIN_PWM_M3_A  = 12;
static const uint8_t PIN_PWM_M3_B  = 13;
static const uint8_t PIN_PWM_M4_A  = 14;
static const uint8_t PIN_PWM_M4_B  = 15;

#define FRONT_MOTORS
//#define BACK_MOTORS

#if defined(FRONT_MOTORS)
static const uint8_t PIN_PWM_A_1   = PIN_PWM_M1_A; 
static const uint8_t PIN_PWM_B_1   = PIN_PWM_M1_B; 
static const uint8_t PIN_PWM_A_2   = PIN_PWM_M2_A; 
static const uint8_t PIN_PWM_B_2   = PIN_PWM_M2_B; 

bool motor1Reverse = true, motor2Reverse = false;

static const uint8_t PIN_ENC_A_1 = 16; 
static const uint8_t PIN_ENC_B_1 = 17; 
static const uint8_t PIN_ENC_A_2 = 18; 
static const uint8_t PIN_ENC_B_2 = 19; 

#elif defined(BACK_MOTORS)
static const uint8_t PIN_PWM_A_1   = PIN_PWM_M3_A; 
static const uint8_t PIN_PWM_B_1   = PIN_PWM_M3_B; 
static const uint8_t PIN_PWM_A_2   = PIN_PWM_M4_A; 
static const uint8_t PIN_PWM_B_2   = PIN_PWM_M4_B; 

static const uint8_t PIN_ENC_A_1 = 26; 
static const uint8_t PIN_ENC_B_1 = 27; 
static const uint8_t PIN_ENC_A_2 = 28; 
static const uint8_t PIN_ENC_B_2 = 29; 

bool motor1Reverse = true, motor2Reverse = false;

#else
#error Need to define FRONT_MOTORS or BACK_MOTORS
#endif

bb::Encoder input[2] = {bb::Encoder(PIN_ENC_A_1, PIN_ENC_B_1), bb::Encoder(PIN_ENC_A_2, PIN_ENC_B_2)};
DCMotor motor[2] = { DCMotor(PIN_PWM_A_1, PIN_PWM_B_1), DCMotor(PIN_PWM_A_2, PIN_PWM_B_2) };

static const float WHEEL_CIRCUMFERENCE = 251.32741229;
static const float WHEEL_TICKS_PER_TURN = 979.2; // 979 ticks per one turn of the drive gear, 18 teeth on the drive gear, 96 teeth on the main gear.
float speedKp = 0.22, speedKi = 1, speedKd = 0.0;
float speedCutoff = 25;
float posKp = 0.05, posKi = 0.0, posKd = 0.0;
float posCutoff = 25;

#else // CONFIG_H
#error Two Config headers included, this is likely not what you want!
#endif