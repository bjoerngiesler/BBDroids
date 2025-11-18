#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130  // Minimum pulse length (0 degrees)
#define SERVOMAX 660  // Maximum pulse length (180 degrees)

void setup(void) {
    Serial.begin(9600);
    while(!Serial);
    Wire.begin();
    if(pwm.begin() == false) {
        Serial.print("Could not find a valid PWM driver, check wiring!\n");
        while(1);
    }
    Serial.print("Found PWM driver\n");

    pwm.setPWMFreq(60);
    for(int i=0; i<16; i++) {
        pwm.setPWM(i, SERVOMIN, SERVOMAX);
        pwm.setPin(i, (SERVOMAX-SERVOMIN)/2);
    }
}

void loop(void) {
    // put your main code here, to run repeatedly:
    delay(1000);
}