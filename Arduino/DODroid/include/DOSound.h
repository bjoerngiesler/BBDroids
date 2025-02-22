#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFPlayerMini_Fast.h>
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
  bool checkSDCard(); // CAREFUL - takes up to 71ms!
  bool sdCardInserted() { return fileCount_ != -1; }
  bool playFolder(Folder folder, int fileNumber);
  bool playFolderRandom(Folder folder);
  bool playSystemSound(int snd);
  bool playSound(int fileNumber);
  bool setVolume(uint8_t vol);

private:
  DFPlayerMini_Fast dfp_;
  bool available_;
  std::map<Folder, int> folderCounts_;
  int fileCount_;
};

#endif // SOUND_H
