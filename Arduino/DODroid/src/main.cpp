#include <Arduino.h>
#include <LibBB.h>
#include <Wire.h>

#include "DODroid.h"
#include "DOSound.h"
#include "DOConfig.h"
#include "DOWifiSecrets.h"
#include "../resources/systemsounds.h"

#include <wiring_private.h>

using namespace bb;

Uart *dfplayerSerial, *serialTXSerial;

bool setupBoardComm() {
  serialTXSerial = new Uart(&sercom3, P_SERIALTX_RX, P_SERIALTX_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_SERIALTX_RX, PIO_SERCOM);
  pinPeripheral(P_SERIALTX_TX, PIO_SERCOM);

  dfplayerSerial = new Uart(&sercom1, P_DFPLAYER_RX, P_DFPLAYER_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  pinPeripheral(P_DFPLAYER_RX, PIO_SERCOM);
  pinPeripheral(P_DFPLAYER_TX, PIO_SERCOM);
  return true;
}

void SERCOM1_Handler() {
  dfplayerSerial->IrqHandler();
}

void SERCOM3_Handler() {
  serialTXSerial->IrqHandler();
}

void initializeSubsystems() {
  ConfigStorage::storage.initialize();
  Runloop::runloop.initialize();
  WifiServer::server.initialize(WIFI_SSID, WIFI_WPA_KEY, WIFI_AP_MODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  WifiServer::server.setOTANameAndPassword("D-O", "OTA");
  XBee::xbee.initialize(DEFAULT_CHAN, DEFAULT_PAN, 115200, serialTXSerial);
  XBee::xbee.setDebugFlags((XBee::DebugFlags)(XBee::DEBUG_PROTOCOL|XBee::DEBUG_XBEE_COMM));
  XBee::xbee.setName(DROID_NAME);
  Servos::servos.initialize();
  Servos::servos.setRequiredIds(std::vector<uint8_t>{SERVO_NECK}); // Rest of the head servos may be disconnected
  Servos::servos.setTorqueOffOnStop(false); // because this can crash the head
  DODroid::droid.initialize();
}

void startSubsystems() {
  WifiServer::server.start();
  XBee::xbee.addPacketReceiver(&DODroid::droid);
  XBee::xbee.start(Console::console.serialStream());
  XBee::xbee.setAPIMode(true);
  Console::console.printfBroadcast("Starting servos\n");
  Servos::servos.start(Console::console.serialStream());
  Console::console.printfBroadcast("Starting droid\n");
  DODroid::droid.start(Console::console.serialStream());
  // sometimes this doesn't work on the first try for whatever reason
  if(WifiServer::server.isStarted() == false) WifiServer::server.start(); 
}

void setup() {
  Serial.begin(2000000);
  Serial.println("Starting up...");

  Wire.begin();
  Wire.setClock(1000000);

  setupBoardComm();

  Console::console.initialize();
  Console::console.start();

  DOSound::sound.begin(dfplayerSerial);

  initializeSubsystems();
  startSubsystems();

  Console::console.printfBroadcast("D-O\n");
  Console::console.printfBroadcast("Firmware version 0.0\n");
  Console::console.printfBroadcast("(c) 2023 Björn Giesler\n");
  Console::console.printfBroadcast("======================\n");

  Runloop::runloop.start();
}

void loop() {}
