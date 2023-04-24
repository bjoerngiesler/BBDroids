#include <LibBB.h>
#include "BB8.h"
#include "BB8Config.h"
#include "BB8WifiSecrets.h"
#include "BB8StatusPixels.h"
#include "BB8Sound.h"
#include "BB8IMU.h"
#include "BB8Servos.h"
#include "BB8Controllers.h"
#include <math.h>
#include <limits.h>
#include <wiring_private.h>

#include <Encoder.h>
#include <WiFiNINA.h>

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;

using namespace bb;

bool setupBoardComm() {
  dynamixelSerial = &Serial1;

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

void setup() {
  Serial.begin(2000000);
  Serial.println(); 
  Serial.println("BB8 Main Board");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  dynamixelSerial = NULL;
  dfplayerSerial = NULL;
  serialTXSerial = NULL;

  setupBoardComm();

  Runloop::runloop.initialize();
  Console::console.initialize();

  XBee::xbee.initialize(DEFAULT_CHAN, 0x3333, DEFAULT_STATION_DROID, DEFAULT_STATION_LEFT_REMOTE, DEFAULT_BPS, serialTXSerial);
  WifiServer::server.initialize(WIFI_SSID, WIFI_WPA_KEY, WIFI_AP_MODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  WifiServer::server.setOTANameAndPassword("BB8-$MAC", "password");
  BB8Servos::servos.initialize();
  BB8::bb8.initialize();

  XBee::xbee.setPacketMode(true);
  XBee::xbee.addPacketReceiver(&BB8::bb8);

  WifiServer::server.start();
  XBee::xbee.start();
  Console::console.start();
  BB8::bb8.start();
  BB8Servos::servos.start();

  BB8StatusPixels::statusPixels.begin();
  BB8Sound::sound.begin(dfplayerSerial);
  BB8PIDController::rollController.begin();
  
  Runloop::runloop.start(); // never returns
}

void loop() {} // not needed - replaced by Runloop::runloop.start()