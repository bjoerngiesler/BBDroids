#include "BB8SerialTX.h"
#include "BB8Config.h"

BB8SerialTX::BB8SerialTX() {
   
}

bool BB8SerialTX::begin(Uart* uart) {
  this->uart = uart; 
#if defined(SERIALTX_MODE_SPEKTRUM)
  Serial.println("Spektrum - running at 125000");
  uart->begin(115200);
  spektrum = new SpektrumRC(*uart);
#elif defined (SERIALTX_MODE_XBEE)
  #error XBEE not supported yet!
#endif

  return true;
}

bool BB8SerialTX::available() {
  return uart->available();
}

bool BB8SerialTX::read() {
#if defined(SERIALTX_MODE_SPEKTRUM)
  return spektrum->read();
#elif defined(SERIAL_TX_MODE_XBEE)
  #error XBEE not supported yet!
#endif
  return false;
}

int16_t BB8SerialTX::channelValue(uint8_t i) {
#if defined(SERIALTX_MODE_SPEKTRUM)
  return map(spektrum->channel((SpektrumRC::ChannelIndex)i), 0, 1023, -511, 512);
#elif defined(SERIAL_TX_MODE_XBEE
  #error XBEE not supported yet!
  return 0;
#endif
}

void BB8SerialTX::printStatus() {
  for(int i=0; i<6; i++) {
    Serial.print("#"); Serial.print(i); Serial.print(": "); Serial.print(channelValue(i)); Serial.print(" ");
  }
  Serial.println();
}
