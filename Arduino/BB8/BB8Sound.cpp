#include "BB8Sound.h"

BB8Sound::BB8Sound() {
}

bool BB8Sound::begin(Uart &ser) {
  Serial.print("Setting up sound... ");
  ser.begin(9600);
  if(dfp.begin(ser)) {
    dfp.play(1);
    Serial.println("OK");
    return true;
  } else {
    Serial.println("Failed.");
    return false;
  }
}
