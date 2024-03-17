#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFRobotDFPlayerMini.h>

class DOSound {
public:
  static DOSound sound;

  DOSound();
  bool begin(Uart *ser);
  bool available() { return available_; }
  bool playFolder(int folderNumber, int fileNumber, bool block = false);
  bool setVolume(uint8_t vol);

private:
  DFRobotDFPlayerMini dfp_;
  bool available_;
};

#endif // SOUND_H
