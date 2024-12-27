/**
 * Example Use of the SpektrumSatellite to receive data:
 * 
 * Please check and adapt the pin assignments for your Microcontroller.
 * If you activate the logging the Serial port used by the SpektrumSatellite must
 * not use Serial (it needs to use e,g Serial1 or Serial2
 * You also might comment out the automatic binding if there is no data because
 * the Receiver needs to be bound only once! If you use the binding please make sure 
 * that you indicate the correct rxPin which is assosiated with the RX pin of the used
 * Serial interface. 
 * 
 */

#include "SpektrumSatellite.h"
#include "SpektrumCSV.h"
#include <wiring_private.h>


int rxPin = 16; // pin for receiving data from serial1

Uart *serialTXSerial;


static const uint8_t P_SERIALTX_TX     = 1;  // Usually the default. Must be usable as Serial TX
static const uint8_t P_SERIALTX_RX     = 0;  // Usually the default. Must be usable as Serial RX

bool setupBoardComm() {
  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);
  new Uart()
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);

  return true;
}

void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}

SpektrumSatellite<uint16_t> *satellite;
SpektrumCSV<uint16_t> csv(',', 0);
uint8_t buffer[1024];

void setup() {
  setupBoardComm();

  serialTXSerial->begin(115200);
  Serial.begin(115200);
  while(!Serial);
  Serial.println();
  Serial.println("setup");

  // Activate the loggin to the console only if SpektrumSatellite is not using Serial
  satellite = new SpektrumSatellite<uint16_t>(*serialTXSerial); // Assing satellite to Serial (use Serial1 or Serial2 if available!)
  satellite->setLog(Serial);
  // we can define the requested binding mode
  satellite->setBindingMode(External_DSM2_22ms);

  //scale the values from 0 to 180 degrees for PWM
  satellite->setChannelValueRange(0, 180);

  // wait forever for data
  satellite->waitForData();
}

void loop() {
  if (satellite->getFrame()) {   
    csv.toString(*satellite, buffer, 1024);
    Serial.print((char*)buffer);     
  }   
}