#include <LibBB.h>
#include "BB8Droid.h"
#include "BB8Config.h"
#include "BB8WifiSecrets.h"
#include "BB8StatusPixels.h"
#include "BB8Sound.h"
#include "BB8Servos.h"
#include <math.h>
#include <limits.h>
#include <wiring_private.h>

#include <Encoder.h>
#include <WiFiNINA.h>

Uart *dynamixelSerial, *dfplayerSerial, *serialTXSerial;

using namespace bb;

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
  // Initialize runloop
  // This can currently go anywhere but just in case that changes, it should be initialized before any other subsystems are.
  Runloop::runloop.initialize();

  // Initialize console. This calls Serial.begin() and opens the default console stream.
  Console::console.initialize();

  // Initialize XBee communication.
  XBee::xbee.initialize(DEFAULT_CHAN, 0x3333, DEFAULT_STATION_DROID, DEFAULT_STATION_LEFT_REMOTE, DEFAULT_BPS, serialTXSerial);
  XBee::xbee.setPacketMode(true);
  XBee::xbee.addPacketReceiver(&BB8::bb8);
  BB8StatusPixels::statusPixels.linkSubsystem(&XBee::xbee, STATUSPIXEL_REMOTE);

  // Initialize Wifi communication.
  WifiServer::server.initialize(WIFI_SSID, WIFI_WPA_KEY, WIFI_AP_MODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  WifiServer::server.setOTANameAndPassword("BB8-$MAC", "password");
  BB8StatusPixels::statusPixels.linkSubsystem(&WifiServer::server, STATUSPIXEL_OVERALL);

  // Initialize the main droid controller.
  BB8::bb8.initialize();
  BB8StatusPixels::statusPixels.linkSubsystem(&BB8::bb8, STATUSPIXEL_OVERALL);

  // Initialize the servos subsystem.
  bb::Servos::servos.initialize();
  BB8StatusPixels::statusPixels.linkSubsystem(&BB8Servos::servos, STATUSPIXEL_MOTORS);
}

void startSubsystems() {
  WifiServer::server.start();
  // for whatever reason this often fails the first time around, so do it again immediately.
  if (!WifiServer::server.isStarted()) WifiServer::server.start();
  BB8StatusPixels::statusPixels.update();
  if (WifiServer::server.isStarted()) BB8Sound::sound.playSystem(SOUND_WIFI_OK);
  else BB8Sound::sound.playSystem(SOUND_WIFI_FAILURE);

  Console::console.start();
  BB8StatusPixels::statusPixels.update();

  XBee::xbee.start();
  BB8StatusPixels::statusPixels.update();
  if (XBee::xbee.isStarted()) BB8Sound::sound.playSystem(SOUND_XBEE_OK);
  else BB8Sound::sound.playSystem(SOUND_XBEE_FAILURE);

  bb::Servos::servos.start();
  BB8StatusPixels::statusPixels.update();
  if (bb::Servos::servos.isStarted()) BB8Sound::sound.playSystem(SOUND_SERVOS_OK);
  else BB8Sound::sound.playSystem(SOUND_SERVOS_FAILURE);

  BB8::bb8.start();
  BB8StatusPixels::statusPixels.update();
  if (BB8::bb8.isStarted()) BB8Sound::sound.playSystem(SOUND_BB8_OK);
  else BB8Sound::sound.playSystem(SOUND_BB8_FAILURE);
}

void setup() {
  Serial.begin(2000000);
  //  while(!Serial);
  Serial.println();
  Serial.println("BB8 Main Board");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  dynamixelSerial = NULL;
  dfplayerSerial = NULL;
  serialTXSerial = NULL;

  setupBoardComm();

  // Set up status pixels first, these are our most basic diagnostics.
  BB8StatusPixels::statusPixels.begin();
  BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_OVERALL, BB8StatusPixels::STATUS_INIT);
  BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_REMOTE, BB8StatusPixels::STATUS_INIT);
  BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_MOTORS, BB8StatusPixels::STATUS_INIT);

  // Next up, sound, so that we can play diagnostics
  BB8Sound::sound.begin(dfplayerSerial);
  BB8StatusPixels::statusPixels.update();
  if (BB8Sound::sound.available()) BB8Sound::sound.playSystem(SOUND_STARTING_UP);

  initializeSubsystems();
  startSubsystems();

  BB8Sound::sound.playSystem(SOUND_MAINLOOP);
  Runloop::runloop.start();  // never returns
}

void loop() {}  // not needed - replaced by Runloop::runloop.start()