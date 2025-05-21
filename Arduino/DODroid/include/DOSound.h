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

  DOSound(uint8_t volume=10);
  void setSerial(Uart *ser);
  bool begin();
  bool available() { return available_; }
  bool checkSDCard(); // CAREFUL - takes up to 71ms!
#if defined(CHECK_SDCARD)
  bool sdCardInserted() { return fileCount_ != -1; }
#else
  bool sdCardInserted() { return true; }
#endif
  bool playFolder(unsigned int folder, unsigned int fileNumber, bool override=false);
  bool playFolderRandom(unsigned int folder);
  bool playFolderNext(unsigned int folder);
  bool playSystemSound(int snd);
  bool playSound(unsigned int fileNumber);
  bool setVolume(uint8_t vol);

private:
  DFPlayerMini_Fast dfp_;
  bool available_;
  bool dumbMode_;
  struct FolderContents {
    unsigned int count;
    unsigned int next;
  };
  uint8_t volume_;

  std::map<unsigned int, FolderContents> folders_;
  int fileCount_;

  Uart *ser_;
};

#endif // SOUND_H
