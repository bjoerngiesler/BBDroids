#include "DOSound.h"
#include "DOConfig.h"

DOSound DOSound::sound;

DOSound::DOSound() {
  available_ = false;
}

bool DOSound::begin(Uart *ser) {
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

bool DOSound::playFolder(int foldernumber, int filenumber, bool block) {
  if(!available_) return false;

  dfp_.playFolder(foldernumber, filenumber);
  if(block) {
    while(dfp_.readState() > 500) delay(1);
  }
  return true;
}

bool DOSound::setVolume(uint8_t vol) {
  if(!available_) return false;
  dfp_.volume(vol);
  return true;
}
