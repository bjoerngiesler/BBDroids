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

Pins pins = leftRemotePins;
bool isLeftRemote;

void setup() {
  Serial.begin(2000000);
  Serial.println("BBDroids Remote Software");
#if !defined(ARDUINO_ARCH_ESP32)
  rp2040.enableDoubleResetBootloader();
#endif
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  isLeftRemote = false;
  pinMode(P_DETECT_WHICH_REMOTE, INPUT);
  digitalWrite(P_DETECT_WHICH_REMOTE, HIGH);
  delay(100);

  uint16_t val = analogRead(P_DETECT_WHICH_REMOTE);
  uint16_t thresh = 500;
  uint16_t min = 2048-thresh, max = 2048+thresh;
  Serial.print("Read "); Serial.print(val);
  if(val < min || val > max) {
    Serial.println(" -- this is the RIGHT remote!");
    pins = rightRemotePins;
  } else {
    Serial.println(" -- this is the LEFT remote!");
    isLeftRemote = true;
    pins = leftRemotePins;
  }
  delay(1000);

  if(isLeftRemote) {
    Serial2.setPins(pins.P_D_DISPLAY_RX, pins.P_D_DISPLAY_TX);
    XBee::xbee.setName("LeftRemote");
  } else {
    pinMode(pins.P_D_RST_IOEXP, OUTPUT);
    digitalWrite(pins.P_D_RST_IOEXP, HIGH);
    XBee::xbee.setName("RightRemote");
  }


  Console::console.initialize();
  Console::console.start();

  Wire.begin();
  Wire.setClock(400000UL);

  ConfigStorage::storage.initialize();
  Runloop::runloop.initialize();

  RDisplay::display.initialize();
  RDisplay::display.start();
  RDisplay::display.setLEDBrightness(16);

  RRemote::remote.initialize();

  uint16_t station = XBee::makeStationID(isLeftRemote ? XBee::REMOTE_BAVARIAN_L : XBee::REMOTE_BAVARIAN_R, BUILDER_ID, REMOTE_ID);
  Serial1.setPins(pins.P_D_XBEE_RX, pins.P_D_XBEE_TX);
  XBee::xbee.initialize(DEFAULT_CHAN, DEFAULT_PAN, station, 115200, &Serial1);

  XBee::xbee.start();
  XBee::xbee.setAPIMode(true);

  Result res = RRemote::remote.start();
  Console::console.printfBroadcast("Result starting remote: %s\n", errorMessage(res));

  XBee::xbee.addPacketReceiver(&RRemote::remote); 
  Runloop::runloop.start(); // never returns
}


void loop() {
}