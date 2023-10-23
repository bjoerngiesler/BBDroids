#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFRobotDFPlayerMini.h>

class BB8Sound {
public:
  static BB8Sound sound;

  BB8Sound();
  bool begin(Uart *ser);
  bool available() { return available_; }
  bool playFolder(int folderNumber, int fileNumber, bool block = false);
  bool playSystem(int fileNumber, bool block = true);
  bool setVolume(uint8_t vol);

private:
  DFRobotDFPlayerMini dfp_;
  bool available_;
};

#endif // SOUND_H
