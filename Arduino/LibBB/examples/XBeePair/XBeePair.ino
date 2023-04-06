#include <Arduino.h>
#include <LibBB8.h>
#include <wiring_private.h>

//#define USE_SERIALTX_REMAP

#ifdef USE_SERIALTX_REMAP

static const uint8_t P_SERIALTX_TX = 0;
static const uint8_t P_SERIALTX_RX = 1;
void SERCOM3_Handler() {
  serialTXSerial.IrqHandler();
}

#endif

void setup(void) {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("Setup");

//  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
//  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);

#ifdef USE_SERIALTX_REMAP
  Uart *serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
#else
  UART *serialTXSerial = new UART(SERIAL1_RX, SERIAL1_TX);
#endif

  BB8XBee::xbee.setDebug(true);
  BB8XBee::xbee.begin(3322, 10, 11, serialTXSerial);
}

void loop(void) {
  delay(1000);
}