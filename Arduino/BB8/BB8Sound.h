#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFRobotDFPlayerMini.h>

class BB8Sound {
public:
  static BB8Sound sound;

  BB8Sound();
  ~BB8Sound();
  bool begin(Uart *ser);
  bool play(int fileNumber = 1);
  bool setVolume(uint8_t vol);

private:
  DFRobotDFPlayerMini *dfp_;
};

#endif // SOUND_H
