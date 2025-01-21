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
    checkSDCard();
    dfp_.volume(30);
    available_ = true;
    return true;
  } 

  return false;
}

bool DOSound::playSound(int fileNumber) {
  if(!available_ || !sdCardInserted()) return false;
  unsigned long us = micros();
  dfp_.play(fileNumber);
  Console::console.printfBroadcast("playFolder() took %dus\n", micros()-us);
  return true;
}

bool DOSound::playFolder(Folder folder, int filenumber) {
  if(!available_ || !sdCardInserted()) return false;

  unsigned long us = micros();
  dfp_.playFolder(int(folder), filenumber);
  Console::console.printfBroadcast("playFolder() took %dus\n", micros()-us);
  return true;
}

bool DOSound::playFolderRandom(Folder folder) {
  if(!available_ || !sdCardInserted()) return false;
  if(folderCounts_.find(folder) == folderCounts_.end()) {
    Console::console.printfBroadcast("No folder with number %d\n", int(folder));
    return false;
  }
  Console::console.printfBroadcast("%d files in folder %d\n", folderCounts_[folder], int(folder));
  int num = random(1, folderCounts_[folder]+1);
  Console::console.printfBroadcast("Playing file %d\n", num);
  return playFolder(folder, num);
}

bool DOSound::playSystemSound(int snd) { 
  if(playFolder(FOLDER_SYSTEM, (int)snd) == true) {
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
  int fd = dfp_.numFolders();

  if(fileCount_ == -1) {
    if(fc == -1) {
      return false; // no change
    }
    fileCount_ = fc;
    Console::console.printfBroadcast("SD card inserted!\n");
    Console::console.printfBroadcast("    %d overall files\n", fileCount_);
    for(int i=1; i<=FOLDER_MAX; i++) {
      int num = dfp_.numTracksInFolder(i);
      Console::console.printfBroadcast("    %d files in folder %d\n", num, i);
      folderCounts_[Folder(i)] = num;
    }
    Console::console.printfBroadcast("Playing initial system sound.\n");
    dfp_.playFolder(int(FOLDER_SYSTEM), SystemSounds::SOUND_SYSTEM_READY);
    dfp_.playFolder(int(FOLDER_SYSTEM), SystemSounds::SOUND_SYSTEM_READY);
    dfp_.playFolder(int(FOLDER_SYSTEM), SystemSounds::SOUND_SYSTEM_READY);
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