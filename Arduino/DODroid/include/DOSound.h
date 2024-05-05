#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFRobotDFPlayerMini.h>

class DOSound {
public:
  static DOSound sound;

  enum SystemSound {
    SYSTEM_SOUND_OK          = 1,
    SYSTEM_SOUND_DEGRADED    = 2,
    SYSTEM_SOUND_UNAVAILABLE = 3,
    SYSTEM_SOUND_POWER       = 10,
    SYSTEM_SOUND_IMU         = 11,
    SYSTEM_SOUND_SERVOS      = 12,
    SYSTEM_SOUND_DRIVESYSTEM = 13,
    SYSTEM_SOUND_STARTUP     = 20,
    SYSTEM_SOUND_SELFTEST    = 21
  };

  DOSound();
  bool begin(Uart *ser);
  bool available() { return available_; }
  bool playFolder(int folderNumber, int fileNumber, bool block = true);
  bool playFolderRandom(int foldernumber, bool block = true);
  bool playSystemSound(int snd, bool block = true) { return playFolder(1, (int)snd, block); }
  bool setVolume(uint8_t vol);

private:
  DFRobotDFPlayerMini dfp_;
  bool available_;
};

#endif // SOUND_H
