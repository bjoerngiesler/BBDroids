#include <Arduino.h>
#include <LibBB8.h>
#include <wiring_private.h>

//#define DROID

#ifdef DROID
static const uint8_t P_SERIALTX_TX = 0;
static const uint8_t P_SERIALTX_RX = 1;
Uart *serialTXSerial;
void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}
#endif

void setup(void) {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Setup");

  BB8XBee::xbee.setDebugFlags((BB8XBee::DebugFlags)(BB8XBee::DEBUG_PROTOCOL));
#ifdef DROID
  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);
  if(BB8XBee::xbee.begin(0xf, 3322, 12, 13, serialTXSerial) == false) {
    Serial.println("Setup failed!");
  } 
#else // Remote
  if(BB8XBee::xbee.begin(0xf, 3322, 13, 12, &Serial1, true) == false) {
    Serial.println("Setup failed!");
  }
#endif

  if(BB8XBee::xbee.healthy()) {
    uint8_t chan; uint16_t pan, station, partner;
    BB8XBee::xbee.getConnectionInfo(chan, pan, station, partner);
    Serial.print("Channel: "); Serial.println(chan);
    Serial.print("PAN:     "); Serial.println(pan);
    Serial.print("Station: "); Serial.println(station);
    Serial.print("Partner: "); Serial.println(partner);
  }

#ifdef DROID
#else
    BB8XBee::xbee.discoverNetworks();
#endif
}

void loop(void) {
#ifndef DROID
  if(!BB8XBee::xbee.healthy()) { 
    Serial.println("Not healthy"); 
  } else if(BB8XBee::xbee.available()) {
    Serial.print("Received: "); Serial.println(BB8XBee::xbee.receive());
  } else {
    Serial.println("Nothing available");
  }
#else
  if(BB8XBee::xbee.healthy()) {
    Serial.println("Sending");
    BB8XBee::xbee.send("Hello\n");
  }
#endif
  delay(1000);
}