#include <LibBB.h>

#include "Config.h"
#include "RRemote.h"
#include "RInput.h"
#include "RDisplay.h"

uint8_t seqnum = 0;

using namespace bb;

int getAnalogReadResolution() { return 12; } // whatever

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

void setup() {
#if !defined(ESP32_REMOTE)
  rp2040.enableDoubleResetBootloader();
#endif
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if !defined(LEFT_REMOTE)
  pinMode(P_D_RST_IOEXP, OUTPUT);
  digitalWrite(P_D_RST_IOEXP, HIGH);
#endif

  Console::console.initialize();
  Console::console.start();
  delay(2000);

  Wire.begin();
  Wire.setClock(400000UL);

  ConfigStorage::storage.initialize();

  Runloop::runloop.initialize();

#if defined(LEFT_REMOTE)
#if defined(ESP32_REMOTE)
  Serial2.setPins(P_D_DISPLAY_RX, P_D_DISPLAY_TX);
#endif
#endif
  RDisplay::display.initialize();

#if defined(LEFT_REMOTE)
  uint16_t station = XBee::makeStationID(XBee::REMOTE_BAVARIAN_L, BUILDER_ID, REMOTE_ID);
#else 
  uint16_t station = XBee::makeStationID(XBee::REMOTE_BAVARIAN_R, BUILDER_ID, REMOTE_ID);
#endif
  Serial1.setPins(P_D_XBEE_RX, P_D_XBEE_TX);
  XBee::xbee.initialize(DEFAULT_CHAN, DEFAULT_PAN, station, 115200, &Serial1);

#if defined(LEFT_REMOTE)
  WifiServer::server.initialize("LRemote-$MAC", "LRemoteKey", true, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
#else
  WifiServer::server.initialize("RRemote-$MAC", "RRemoteKey", true, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
#endif
  RRemote::remote.initialize();

  RDisplay::display.start();
#if defined(LEFT_REMOTE)
  XBee::xbee.setName("LeftRemote");
#else
  XBee::xbee.setName("RightRemote");
#endif
  WifiServer::server.start();
  XBee::xbee.start();
  XBee::xbee.setAPIMode(true);

  Result res = RRemote::remote.start();
  Console::console.printfBroadcast("Result starting remote: %s\n", errorMessage(res));

  XBee::xbee.addPacketReceiver(&RRemote::remote); 
  Runloop::runloop.start(); // never returns
}


void loop() {
}