#include "DOSound.h"
#include <LibBB.h>

using namespace bb;

DOSound DOSound::sound;

DOSound::DOSound() {
  available_ = false;
  fileCountsSetup_ = false;
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
    int errcode = dfp_.readType(); 
    Serial.print("error code ");
    Serial.print(errcode);
    Serial.println("... failed!");
    if(errcode != 0) {
      available_ = false;
      return false;
    } else {
      available_ = true;
      return true;
    }
  }

}

bool DOSound::playSound(int fileNumber) {
  if(!available_) return false;
  dfp_.play(fileNumber);
  return true;
}


bool DOSound::playFolder(Folder folder, int filenumber, bool block) {
  return true;

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
  return true;

  if(!available_) return false;
  if(!fileCountsSetup_) setupFileCounts();

  Console::console.printfBroadcast("%d files in folder %d\n", fileCounts_[folder], folder);
  int num = random(1, fileCounts_[folder]+1);
  Console::console.printfBroadcast("Playing file %d\n", num);
  return playFolder(folder, num, block);
}


bool DOSound::setVolume(uint8_t vol) {
  if(!available_) return false;
  dfp_.volume(vol);
  return true;
}

void DOSound::setupFileCounts() {
  Serial.print("Reading file counts...");
  for(int i=1; i<=FOLDER_MAX; i++) {
#if 0
    int num;
    num = dfp_.readFileCountsInFolder(i);
    num = dfp_.readFileCountsInFolder(i);
    num = dfp_.readFileCountsInFolder(i);
    Serial.print(" Folder ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(num);

    fileCounts_[Folder(i)] = num;
#endif
    fileCounts_[Folder(i)] = 0;
  }

  fileCountsSetup_ = true;
}