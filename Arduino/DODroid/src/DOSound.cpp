#include "DOSound.h"
#include <LibBB.h>
#include "../resources/systemsounds.h"

using namespace bb;

DOSound DOSound::sound;
#define DOSOUND_REPEATS 1
#define DOSOUND_DELAY_US 100

DOSound::DOSound(uint8_t volume) {
  available_ = false;
  dumbMode_ = false;
  fileCount_ = -1;
  ser_ = nullptr;
  volume_ = volume;
}

void DOSound::setSerial(Uart *ser) {
  ser_ = ser;
}

bool DOSound::begin() {  
  if(ser_ == NULL) {
    bb::printf("Serial is NULL!\n"); 
    available_ = false;
    return false;
  } 

  bb::printf("Setting up sound... ");
  ser_->begin(9600);
  if(dfp_.begin(*ser_)) {
    for(int i=0; i<3; i++) {
      if(checkSDCard() == true) break;
      delay(1000);
    }
    for(int i=0; i<DOSOUND_REPEATS; i++) {
      dfp_.volume(volume_);
      delayMicroseconds(DOSOUND_DELAY_US);
    }
    available_ = true;
    bb::printf("OK\n");
    return true;
  } 
  bb::printf("failure\n");
  return false;
}

bool DOSound::playSound(unsigned int fileNumber) {
  if(!available_) {
    bb::printf("Sound system not available!\n");
    return false;
  }

  if(dumbMode_ == false && sdCardInserted() == false) {
    bb::printf("SD card not inserted!\n");
    return false;
  } 

  for(int i=0; i<DOSOUND_REPEATS; i++) {
    dfp_.play(fileNumber);
    delayMicroseconds(DOSOUND_DELAY_US);
  }
  return true;
}

bool DOSound::playFolder(unsigned int folder, unsigned int filenumber, bool override) {
  if(!available_) {
    bb::printf("Sound system not available!\n");
    if(override == false) return false;
  }

  if(dumbMode_ == false && sdCardInserted() == false) {
    bb::printf("SD card not inserted!\n");
    if(override == false) return false;
  } 

  for(int i=0; i<DOSOUND_REPEATS; i++) {
    dfp_.playFolder(int(folder), filenumber);
    delayMicroseconds(DOSOUND_DELAY_US);
  }
  return true;
}

bool DOSound::playFolderRandom(unsigned int folder) {
  if(!available_) {
    bb::printf("Sound system not available!\n");
    return false;
  }

  if(dumbMode_ == false) {
    if(sdCardInserted() == false) {
      bb::printf("SD card not inserted!\n");
      return false;
    } 

    if(folders_.find(folder) == folders_.end()) {
      bb::printf("No folder with number %d\n", int(folder));
      return false;
    }
    bb::printf("%d files in folder %d\n", folders_[folder].count, int(folder));
    int num = random(0, folders_[folder].count);
    bb::printf("Playing file %d\n", num);
    if(playFolder(folder, num+1) == true) {
      folders_[folder].next = (folders_[folder].next+1)%folders_[folder].count;
      return true;
    } else return false;
  } else { // dumbMode_ == true
    bb::printf("Dumb mode, playing first file in folder %d\n", folder);
    return playFolder(folder, 1);
  }
}

bool DOSound::playFolderNext(unsigned int folder) {
  if(!available_) {
    bb::printf("Sound system not available!\n");
    return false;
  }

  if(dumbMode_ == false) {
    if(sdCardInserted() == false) {
      bb::printf("SD card not inserted!\n");
      return false;
    } 
    if(folders_.find(folder) == folders_.end()) {
      bb::printf("No folder with number %d\n", int(folder));
      return false;
    }

    bb::printf("%d files in folder %d\n", folders_[folder].count, int(folder));
    bb::printf("Playing file %d\n", folders_[folder].next);

    if(playFolder(folder, folders_[folder].next+1) == true) {
      folders_[folder].next = (folders_[folder].next+1)%folders_[folder].count;
      return true;
    } else return false;
  } else { // dumbMode_ == true
    bb::printf("Dumb mode, playing first file in folder %d\n", folder);
    return playFolder(folder, 1);
  }
}

bool DOSound::playSystemSound(int snd) { 
  if(playFolder(1, (int)snd, true) == true) {
    delay(1200);
    return true;
  }
  return false;
}

bool DOSound::setVolume(uint8_t vol) {
  if(!available_) return false;
  if(vol == volume_) return false;
  for(int i=0; i<DOSOUND_REPEATS; i++) {
    dfp_.volume(vol);
    delayMicroseconds(DOSOUND_DELAY_US);
  }
  volume_ = vol;
  return true;
}

bool DOSound::checkSDCard() {
  if(dumbMode_) return false;

  int fc = -1;
  for(int i=0; i<DOSOUND_REPEATS; i++) {
    int fc_ = dfp_.numSdTracks();
    if(fc_ != -1) {
      fc = fc_;
      break;
    }
    delayMicroseconds(DOSOUND_DELAY_US);
  }

  if(fileCount_ == -1) {
    if(fc == -1) {
      return false; // no change
    }
    fileCount_ = fc;
    bb::printf("SD card inserted!\n");
    bb::printf("    %d overall files\n", fileCount_);
    for(unsigned int i=1; i<=8; i++) {
      int num = dfp_.numTracksInFolder(i);
      bb::printf("    %d files in folder %d\n", num, i);
      if(num > 0)
        folders_[i] = {unsigned(num), 0};
    }

    if(fileCount_ > 0 && folders_.size() == 0) {
      bb::printf("No folders found. Bad DFPlayer or wrong folder structure. Entering dumb mode.\n");
      dumbMode_ = true;
    } else {
      dumbMode_ = false;
    }

    bb::printf("Playing initial system sound %d.\n", SystemSounds::SOUND_SYSTEM_READY);
    dfp_.playFolder(int(1), SystemSounds::SOUND_SYSTEM_READY);
    delay(1000);
    bb::printf("Done.\n");
    return true;
  } else {
    if(fc == fileCount_) return true; // no change
    if(fc == -1) {
      bb::printf("SD card removed!\n");
      fileCount_ = -1;
      return false;
    }
  }
  return false;
}