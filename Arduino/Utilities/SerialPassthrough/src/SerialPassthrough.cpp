#include <Arduino.h>
#include <vector>
#include <algorithm>
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
  Serial.println("SerialPassthrough");
  Serial.println("=================");
  Serial.println("This program will pass through all characters typed on the console serial port to the");
  Serial.println(String("serial port at RX=") + P_PASSTHROUGH_RX + ", TX=" + P_PASSTHROUGH_TX + "and vice versa.");
  
  while(true) {
    Serial.print("Please enter baudrate to open the port at: ");
    String answer = Serial.readStringUntil('\n');
    int baud = answer.toInt();
    if(baud <= 0) {
      Serial.println("Error: Baud must be larger than zero.");
      continue;
    }
    if(std::find(standardBaudRates.begin(), standardBaudRates.end(), baud) == standardBaudRates.end()) {
      Serial.println("Nonstandard baud rate, trying anyway.");
    }

    passthroughSerial.begin(baud);
    break;
  }
}

void loop() {
  while(Serial.available()) passthroughSerial.write(Serial.read());
  while(passthroughSerial.available()) Serial.write(passthroughSerial.read());
}
