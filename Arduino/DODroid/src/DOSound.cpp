#include "DOSound.h"
#include <LibBB.h>
#include "../resources/systemsounds.h"

using namespace bb;

DOSound DOSound::sound;

DOSound::DOSound() {
  available_ = false;
  fileCount_ = -1;
}

bool DOSound::begin(Uart *ser) {
  if(ser == NULL) {
    Console::console.printfBroadcast("Serial is NULL!\n"); 
    available_ = false;
    return false;
  } 

  Console::console.printfBroadcast("Setting up sound...\n");
  ser->begin(9600);
  if(dfp_.begin(*ser)) {
    for(int i=0; i<3; i++) {
      if(checkSDCard() == true) break;
      delay(1000);
    }
    dfp_.volume(10);
    available_ = true;
    return true;
  } 

  return false;
}

bool DOSound::playSound(unsigned int fileNumber) {
  if(!available_ || !sdCardInserted()) return false;
  dfp_.play(fileNumber);
  return true;
}

bool DOSound::playFolder(unsigned int folder, unsigned int filenumber) {
  if(!available_ || !sdCardInserted()) return false;
  dfp_.playFolder(int(folder), filenumber);
  return true;
}

bool DOSound::playFolderRandom(unsigned int folder) {
  if(!available_ || !sdCardInserted()) return false;
  if(folders_.find(folder) == folders_.end()) {
    Console::console.printfBroadcast("No folder with number %d\n", int(folder));
    return false;
  }
  Console::console.printfBroadcast("%d files in folder %d\n", folders_[folder].count, int(folder));
  int num = random(0, folders_[folder].count);
  Console::console.printfBroadcast("Playing file %d\n", num);
  if(playFolder(folder, num+1) == true) {
    folders_[folder].next = (folders_[folder].next+1)%folders_[folder].count;
    return true;
  } else return false;
}

bool DOSound::playFolderNext(unsigned int folder) {
  if(!available_ || !sdCardInserted()) return false;
  if(folders_.find(folder) == folders_.end()) {
    Console::console.printfBroadcast("No folder with number %d\n", int(folder));
    return false;
  }
  Console::console.printfBroadcast("%d files in folder %d\n", folders_[folder].count, int(folder));
  Console::console.printfBroadcast("Playing file %d\n", folders_[folder].next);
  if(playFolder(folder, folders_[folder].next+1) == true) {
    folders_[folder].next = (folders_[folder].next+1)%folders_[folder].count;
    return true;
  } else return false;
}

bool DOSound::playSystemSound(int snd) { 
  if(playFolder(1, (int)snd) == true) {
    delay(1200);
    return true;
  }
  return false;
}

bool DOSound::setVolume(uint8_t vol) {
  if(!available_) return false;
  dfp_.volume(vol);
  return true;
}

bool DOSound::checkSDCard() {
  int fc = dfp_.numSdTracks();

  if(fileCount_ == -1) {
    if(fc == -1) {
      return false; // no change
    }
    fileCount_ = fc;
    Console::console.printfBroadcast("SD card inserted!\n");
    Console::console.printfBroadcast("    %d overall files\n", fileCount_);
    for(unsigned int i=1; i<=8; i++) {
      int num = dfp_.numTracksInFolder(i);
      Console::console.printfBroadcast("    %d files in folder %d\n", num, i);
      if(num > 0)
        folders_[i] = {num, 0};
    }
    Console::console.printfBroadcast("Playing initial system sound.\n");
    dfp_.playFolder(int(1), SystemSounds::SOUND_SYSTEM_READY);
    dfp_.playFolder(int(1), SystemSounds::SOUND_SYSTEM_READY);
    dfp_.playFolder(int(1), SystemSounds::SOUND_SYSTEM_READY);
    Console::console.printfBroadcast("Done.\n");
    return true;
  } else {
    if(fc == fileCount_) return true; // no change
    if(fc == -1) {
      Console::console.printfBroadcast("SD card removed!\n");
      fileCount_ = -1;
      return false;
    }
  }
  return false;
}