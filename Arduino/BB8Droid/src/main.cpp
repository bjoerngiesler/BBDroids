#include <LibBB.h>
#include "BB8Droid.h"
#include "BB8Config.h"
#include "BB8WifiSecrets.h"
#include "BB8StatusPixels.h"
#include "BB8Sound.h"
#include <math.h>
#include <limits.h>
#include <wiring_private.h>

using namespace bb;

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;

void setupBoardComm() {
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
  ConfigStorage::storage.initialize();
  Runloop::runloop.initialize();
  Console::console.initialize();
  WifiServer::server.initialize(WIFI_SSID, WIFI_WPA_KEY, WIFI_AP_MODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  WifiServer::server.setOTANameAndPassword("BB8-$MAC", "password");

  XBee::xbee.initialize(DEFAULT_CHAN, DEFAULT_PAN, 230400, serialTXSerial);
  XBee::xbee.setDebugFlags((XBee::DebugFlags)(XBee::DEBUG_PROTOCOL|XBee::DEBUG_XBEE_COMM));
  XBee::xbee.setName(DROID_NAME);

  Servos::servos.initialize();
  Servos::servos.setRequiredIds({DOME_HEADING_SERVO, DOME_ROLL_SERVO, DOME_PITCH_SERVO, BODY_ROLL_SERVO});
  Servos::servos.setTorqueOffOnStop(true);

  BB8::bb8.initialize();
}

void startSubsystems() {
  Console::console.start();
  WifiServer::server.start();
  XBee::xbee.addPacketReceiver(&BB8::bb8);
  XBee::xbee.start();
  XBee::xbee.setAPIMode(true);
  bb::Servos::servos.start();
  BB8::bb8.start();
  if (!WifiServer::server.isStarted()) WifiServer::server.start();
}

void setup() {
  Serial.begin(2000000);
  //  while(!Serial);
  Serial.println();

  Wire.begin();
  setupBoardComm();

  initializeSubsystems();
  startSubsystems();

  Console::console.printfBroadcast("BB8 Main Board\n");
  Console::console.printfBroadcast("Firmware Version 0.0\n");
  Console::console.printfBroadcast("(c) 2022 Felix Beyer, Björn Giesler\n");
  Console::console.printfBroadcast("===================================\n");

  Runloop::runloop.start();  // never returns
}

void loop() {}  // not needed - replaced by Runloop::runloop.start()