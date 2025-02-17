#include <Arduino.h>
#include <vector>
#include <algorithm>
#include <LibBB.h>
#include <wiring_private.h>

std::vector<int> standardBaudRates = {9600, 19200, 28800, 38400, 57600, 76800, 115200};

const uint8_t P_PASSTHROUGH_RX = 1;
const uint8_t P_PASSTHROUGH_TX = 0;
Uart passthroughSerial(&sercom3, P_PASSTHROUGH_RX, P_PASSTHROUGH_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler() {
  passthroughSerial.IrqHandler();
}

void setup() {
  Serial.begin(2000000);
  while(!Serial);
  
  pinPeripheral(P_PASSTHROUGH_RX, PIO_SERCOM);
  pinPeripheral(P_PASSTHROUGH_TX, PIO_SERCOM);

  Serial.println();
  Serial.println("SerialPassthrough");
  Serial.println("=================");
  Serial.println("This program will pass through all characters typed on the console serial port");
  Serial.println(String("to the serial port at RX=") + P_PASSTHROUGH_RX + ", TX=" + P_PASSTHROUGH_TX + " and vice versa.");
  Serial.println("Does not echo; turn local echo on in your terminal if required.");
}

void begin() {
  int baud;

  Serial.println();
  while(true) {
    String answer;
    Serial.println("Please enter baudrate to use:");
    Serial.print("> ");
    while(bb::SerialConsoleStream::readStringUntil(Serial, '\n', answer) == false);

    baud = answer.toInt();
    if(baud <= 0) {
      Serial.println("Error: Baud must be larger than zero.");
      continue;
    }
    break;
  }

  if(std::find(standardBaudRates.begin(), standardBaudRates.end(), baud) == standardBaudRates.end()) {
    Serial.println("Nonstandard baud rate, trying anyway.");
  }

  Serial.println("Passing all characters through from now on. Type ESC to exit.");
  passthroughSerial.begin(baud);
}

void loop() {
  begin();
  while(true) {
    uint8_t c;

    while(passthroughSerial.available()) {
      c = passthroughSerial.read();
      Serial.write(c);
      if(c == '\r') Serial.write('\n');
    }

    while(Serial.available()) {
      uint8_t c = Serial.read();
      if(c == 0x1b) { // escape
        Serial.println();
        Serial.println("ESC caught, exiting.");
        return;
      }
      passthroughSerial.write(c);
    }
  }
}
