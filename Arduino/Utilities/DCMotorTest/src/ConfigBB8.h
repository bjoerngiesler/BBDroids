#if !defined(CONFIG_H)
#define CONFIG_H

static const uint8_t PIN_DIR_A_1   = 2;
static const uint8_t PIN_DIR_B_1   = 3;
static const uint8_t PIN_DIR_PWM_1 = 19;
static const uint8_t PIN_ENABLE_1  = 20;
static const uint8_t PIN_DIR_A_2   = PIN_DISABLE;
static const uint8_t PIN_DIR_B_2   = PIN_DISABLE;
static const uint8_t PIN_DIR_PWM_2 = PIN_DISABLE;
static const uint8_t PIN_ENABLE_2  = PIN_DISABLE


static const uint8_t PIN_ENC_A_1 = 17;
static const uint8_t PIN_ENC_B_1 = 16;
static const uint8_t PIN_ENC_A_2 = PIN_DISABLE;
static const uint8_t PIN_ENC_B_2 = PIN_DISABLE;

DCMotor motor[2] = { DCMotor(PIN_DIR_A_1, PIN_DIR_B_1, PIN_DIR_PWM_1, PIN_ENABLE_1), DCMotor(PIN_DIR_A_2, PIN_DIR_B_2, PIN_DIR_PWM_2, PIN_ENABLE_2) };
bb::Encoder input[2] = {bb::Encoder(PIN_ENC_A_1, PIN_ENC_B_1), bb::Encoder(PIN_ENC_A_2, PIN_ENC_B_2)};

// Values for BB-8. This gives about 0.33mm per tick.
static const float WHEEL_CIRCUMFERENCE     = 2*M_PI*253.0; // bb8 motor pwm 0
static const float WHEEL_TICKS_PER_TURN    = 4776.384;
float speedKp = 0.075, speedKi = 0.2, speedKd = 0.0;
float posKp = 0.027, posKi = 0.005, posKd = 0.0;
float speedCutoff = 25;
float posCutoff = 25;

bool motor1Reverse = false, motor2Reverse = false;

#else // CONFIG_H
#error Two Config headers included, this is likely not what you want!
#endif