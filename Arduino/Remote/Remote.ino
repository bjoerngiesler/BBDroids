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

  Serial.begin(2000000);
  //while(!Serial);

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
  XBee::xbee.initialize(DEFAULT_CHAN, DEFAULT_PAN, station, DEFAULT_STATION_DROID, 115200);
  
#if defined(LEFT_REMOTE)
  WifiServer::server.initialize("LRemote-$MAC", "LRemoteKey", true, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
#else
  WifiServer::server.initialize("RRemote-$MAC", "RRemoteKey", true, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
#endif
  RRemote::remote.initialize();

#if defined(LEFT_REMOTE)
  RDisplay::display.start();
  XBee::xbee.setName("LeftRemote");
#else
  XBee::xbee.setName("RightRemote");
#endif
  WifiServer::server.start();
  XBee::xbee.start();
  XBee::xbee.setAPIMode(true);
  
  RRemote::remote.start();

  XBee::xbee.addPacketReceiver(&RRemote::remote);
  Runloop::runloop.start(); // never returns
}


void loop() {
}
