#include <LibBB.h>

#include "DODroid.h"
#include "DOConfig.h"
#include "DOWifiSecrets.h"

#include <wiring_private.h>

using namespace bb;

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;

bool setupBoardComm() {
  dynamixelSerial = &Serial1;

  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);

  dfplayerSerial = new Uart(&sercom1, P_DFPLAYER_RX, P_DFPLAYER_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_DFPLAYER_RX, PIO_SERCOM);
  pinPeripheral(P_DFPLAYER_TX, PIO_SERCOM);
}

void SERCOM1_Handler() {
  dfplayerSerial->IrqHandler();
}

void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}

void initializeSubsystems() {
  Runloop::runloop.initialize();
  Console::console.initialize();
  WifiServer::server.initialize(WIFI_SSID, WIFI_WPA_KEY, WIFI_AP_MODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  DODroid::droid.initialize();
}

void startSubsystems() {
  WifiServer::server.start();
  Console::console.start();
  DODroid::droid.start();
}

void setup() {
  Serial.begin(2000000);
  Serial.println();
  Serial.println("D-O");
  Serial.println("Firmware version 0.0");
  Serial.println("(c) 2023 Björn Giesler");
  Serial.println("======================");

  setupBoardComm();

  initializeSubsystems();
  startSubsystems();
  Runloop::runloop.start();
}

void loop() {}