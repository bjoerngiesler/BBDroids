#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFRobotDFPlayerMini.h>
#include <map>

class DOSound {
public:
  static DOSound sound;

  enum Folder {
    FOLDER_SYSTEM   = 1,
    FOLDER_GREETING = 2,
    FOLDER_POSITIVE = 3,
    FOLDER_NEGATIVE = 4,
    FOLDER_CURIOUS  = 5
  };

  static const uint8_t FOLDER_MAX = 5;

  DOSound();
  bool begin(Uart *ser);
  bool available() { return available_; }
  bool playFolder(Folder folder, int fileNumber, bool block = true);
  bool playFolderRandom(Folder folder, bool block = true);
  bool playSystemSound(int snd, bool block = true) { return playFolder(FOLDER_SYSTEM, (int)snd, block); }
  bool playSound(int fileNumber);
  bool setVolume(uint8_t vol);
  void setupFileCounts();

private:
  DFRobotDFPlayerMini dfp_;
  bool available_;
  std::map<Folder, int> fileCounts_;
  bool fileCountsSetup_;
};

#endif // SOUND_H
