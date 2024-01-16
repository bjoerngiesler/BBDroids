#include <LibBB.h>
#include <WiFiNINA.h>

#include "Config.h"
#include "RRemote.h"
#include "RemoteInput.h"
#include "RDisplay.h"
#include "RMenu.h"
#include "IMUFilter.h"

uint8_t seqnum = 0;

using namespace bb;

int getAnalogReadResolution() { return 12; } // whatever

void setup() {
  rp2040.enableDoubleResetBootloader();

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);

  Console::console.initialize();
  Console::console.start();
  
  Runloop::runloop.initialize();
#if defined(LEFT_REMOTE)
  RDisplay::display.initialize();
  uint16_t station = XBee::makeStationID(XBee::REMOTE_BAVARIAN_L, BUILDER_ID, REMOTE_ID);
#else 
  uint16_t station = XBee::makeStationID(XBee::REMOTE_BAVARIAN_R, BUILDER_ID, REMOTE_ID);
#endif
  XBee::xbee.initialize(DEFAULT_CHAN, 0x3333, station, DEFAULT_STATION_DROID, DEFAULT_BPS);
  XBee::xbee.setPacketMode(true);
  WifiServer::server.initialize("Hogwarts", "1s(h1pu+Bj0rn", false, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  RRemote::remote.initialize();

#if defined(LEFT_REMOTE)
  RDisplay::display.start();
  XBee::xbee.setName("LeftRemote");
#else
  XBee::xbee.setName("RightRemote");
#endif
  WifiServer::server.start();
  XBee::xbee.start();
  RRemote::remote.start();

#if defined(LEFT_REMOTE)
  XBee::xbee.addPacketReceiver(&RRemote::remote);
#endif
  Runloop::runloop.start(); // never returns
}


void loop() {
}
