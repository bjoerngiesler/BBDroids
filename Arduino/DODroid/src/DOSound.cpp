#include "DOSound.h"
#include <LibBB.h>

using namespace bb;

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
    dfp_.volume(30);
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

bool DOSound::playFolder(Folder folder, int filenumber, bool block) {
  if(!available_) return false;

  dfp_.playFolder(int(folder), filenumber);
  if(block) {
    delay(1000);
#if 0
    bool debug = false;
    delay(200);
    int timeout = 500, state = 0;
    if(debug) Serial.print("Wait for start...");
    while(state != 513 && timeout >= 0) {
      delay(100);
      if(debug) Serial.print(".");
      state = dfp_.readState();
      timeout -= 100;
    }
    if(timeout < 0) {
      if(debug) Serial.println(" timeout.");
      return false;
    }
   if(debug) Serial.println(" started.");
    
    if(debug) Serial.print("Wait for end...");
    timeout = 10000;
    while(state == 513) {
      delay(200);
      if(debug) Serial.print(".");
      state = dfp_.readState();
      timeout -= 200;
    }
    if(timeout < 0) {
      if(debug) Serial.println(" timeout");
      return false;
    }
    if(debug) Serial.println(" ended.");
#endif
  }
  return true;
}

bool DOSound::playFolderRandom(Folder folder, bool block) {
  return false;
  if(!available_) return false;

  int num = dfp_.readFileCountsInFolder(int(folder));
  if(num == 0) {
    Console::console.printfBroadcast("Folder %d empty!\n", num);
    return false;
  }
  num = random(1, num);
  Console::console.printfBroadcast("Playing folder %d file %d", int(folder), num);
  return playFolder(folder, num, block);
}


bool DOSound::setVolume(uint8_t vol) {
  if(!available_) return false;
  dfp_.volume(vol);
  return true;
}
