#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFPlayerMini_Fast.h>
#include <map>

#define CHECK_SDCARD

class DOSound {
public:
  static DOSound sound;

  DOSound();
  bool begin(Uart *ser);
  bool available() { return available_; }
  bool checkSDCard(); // CAREFUL - takes up to 71ms!
#if defined(CHECK_SDCARD)
  bool sdCardInserted() { return fileCount_ != -1; }
#else
  bool sdCardInserted() { return true; }
#endif
  bool playFolder(unsigned int folder, unsigned int fileNumber);
  bool playFolderRandom(unsigned int folder);
  bool playFolderNext(unsigned int folder);
  bool playSystemSound(int snd);
  bool playSound(unsigned int fileNumber);
  bool setVolume(uint8_t vol);

private:
  DFPlayerMini_Fast dfp_;
  bool available_;
  struct FolderContents {
    unsigned int count;
    unsigned int next;
  };

  std::map<unsigned int, FolderContents> folders_;
  int fileCount_;
};

#endif // SOUND_H
