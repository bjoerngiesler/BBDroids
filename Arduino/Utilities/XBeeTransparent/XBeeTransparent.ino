#include <Arduino.h>
#include <wiring_private.h>

#define FULLY_TRANSPARENT
#define DROID
#define BAUD 9600

#ifdef DROID
static const uint8_t P_SERIALTX_TX = 0;
static const uint8_t P_SERIALTX_RX = 1;
Uart *serialTXSerial;
void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}
#else
HardwareSerial *serialTXSerial;
#endif

void setup(void) {
#ifdef FULLY_TRANSPARENT
  Serial.begin(BAUD);
#else
  Serial.begin(BAUD);
  while(!Serial);
  Serial.println();
  Serial.println("Enter '+++' for AT mode. Use Carriage Return as line end.");
#endif

#ifdef DROID
  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);
#else // Remote
   serialTXSerial = &Serial1;
 #endif

  serialTXSerial->begin(BAUD);
}

void loop(void) {
#ifdef FULLY_TRANSPARENT
  while(serialTXSerial->available()) Serial.write(serialTXSerial->read());
  while(Serial.available()) serialTXSerial->write(Serial.read());
  
#else

  while(serialTXSerial->available()) {
    uint8_t c = serialTXSerial->read();
    Serial.write(c);
    if(c == '\r') Serial.write('\n');
  }

  while(Serial.available()) {
    String str = Serial.readString();
    Serial.print("> "); Serial.println(str);
    if(str.startsWith("+++")) {
      serialTXSerial->print("+++");
    } else {
      serialTXSerial->println(str);
    }
  }
#endif
}