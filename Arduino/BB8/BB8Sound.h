#if !defined(SOUND_H)
#define SOUND_H

#include <Arduino.h>
#include <wiring_private.h>
#include <DFRobotDFPlayerMini.h>

class BB8Sound {
  public:
  BB8Sound();
  bool begin(Uart &ser);

  private:
  DFRobotDFPlayerMini dfp;
};

#endif // SOUND_H
