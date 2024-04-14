#include "BB8Sound.h"
#include "BB8Config.h"

BB8Sound BB8Sound::sound;

BB8Sound::BB8Sound() {
  available_ = false;
}

bool BB8Sound::begin(Uart *ser) {
  if(ser == NULL) {
    Serial.println("Serial is NULL!"); 
    available_ = false;
    return false;
  } 

  Serial.print("Setting up sound... ");
  ser->begin(9600);
  if(dfp_.begin(*ser)) {
    dfp_.volume(10);
    Serial.println("success.");
    available_ = true;
    return true;
  } else {
    Serial.print("error code ");
    Serial.print(dfp_.readType());
    Serial.println("... failed!");
    available_ = false;
    return false;
  }
}

bool BB8Sound::playFolder(int foldernumber, int filenumber, bool block) {
  if(!available_) return false;

  dfp_.playFolder(foldernumber, filenumber);
  if(block) {
    while(dfp_.readState() > 500) delay(1);
  }
  return true;
}

bool BB8Sound::playSystem(int filenumber, bool block) {
  return playFolder(SOUND_FOLDER_SYSTEM, filenumber, block);
}

bool BB8Sound::setVolume(uint8_t vol) {
  if(!available_) return false;
  dfp_.volume(vol);
  return true;
}
