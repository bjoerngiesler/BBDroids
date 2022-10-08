#if !defined(BB8SERIALTX_H)
#define BB8SERIALTX_H

#include <Arduino.h>
#include <wiring_private.h>
#include "BB8Config.h"


#if defined(SERIALTX_MODE_SPEKTRUM) && defined(SERIALTX_MODE_XBEE)
#error Please only uncomment either SERIALTX_MODE_SPEKTRUM or SERIALTX_MODE_XBEE in Config.h!
#endif

#if defined(SERIALTX_MODE_SPEKTRUM)
#include <SpektrumRC.h>
#endif

class BB8SerialTX {
public:

  BB8SerialTX();
  bool begin(Uart* uart);

  bool available();
  bool read();
  void printStatus();

  int16_t channelValue(uint8_t i);

private:
  Uart* uart;
#if defined(SERIALTX_MODE_SPEKTRUM)
  SpektrumRC *spektrum;
#endif
};

#endif //BB8SERIALTX_H
