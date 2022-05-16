#include "BB8SerialTX.h"
#include "Config.h"

#if defined(SERIALTX_MODE_SPEKTRUM) && defined(SERIALTX_MODE_XBEE)
#error Please only uncomment either SERIALTX_MODE_SPEKTRUM or SERIALTX_MODE_XBEE in Config.h!
#endif

#if defined(SERIALTX_MODE_SPEKTRUM)
#include <SpektrumRC.h>
#endif

BB8SerialTX::BB8SerialTX() {
   
}

bool BB8SerialTX::begin(Uart* uart) {
  this->uart = uart; 
#if defined(SERIALTX_MODE_SPEKTRUM)
  Serial.println("Spektrum - running at 125000");
  uart->begin(125000);
#elif defined (SERIALTX_MODE_XBEE)
  #error XBEE not supported yet!
#endif

  return true;
}

bool BB8SerialTX::available() {
  return uart->available();
}

void BB8SerialTX::read() {
  if(!uart->available()) {
    Serial.println("nothinng available");
    return;
  }

  while(uart->available()) {
    for(int i=0; i<16; i++) {
      Serial.print(uart->read()); Serial.print(" ");
    }
    Serial.println();
  }
}
