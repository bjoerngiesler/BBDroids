#include "BB8Sound.h"

BB8Sound::BB8Sound() {
}

bool BB8Sound::begin(Uart &ser) {
  Serial.print("Setting up sound... ");
  ser.begin(9600);
  if(dfp.begin(ser)) {
    dfp.volume(25);
    dfp.play(1);
    Serial.println("success.");
    return true;
  } else {
    Serial.println("failed!");
    return false;
  }
}
