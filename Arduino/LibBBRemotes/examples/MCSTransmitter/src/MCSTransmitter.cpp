/*
  BBRemoteTransmitter example
  2025 by Björn Giesler
  This example realizes a simple 3-axis remote control using the Bavarian Builders' 
  BBRemotes framework to communicate over various communication mechanisms.
  HOW TO USE:
    1. Choose what communication protocol you want to use, and uncomment the respective
       lines in the protocol section.
    2. Modify the pin mappings in the pins section to reflect the pins you have connected
       the hardware (joysticks, pots, buttons) to.
*/

#include <LibBBRemotes.h>

using namespace bb::rmt;

// Uncomment the protocol you want to use
ProtocolType type = 
  // MONACO_XBEE;
  MONACO_ESPNOW;
  // MONACO_BLE;
  // MONACO_UDP;
  // SPHERO_BLE;
  // DROIDDEPOT_BLE;
  // SPEKTRUM_DSSS;

// These input pins are used to read the remote hardware. Modify them so they reflect
// how you connect your joystick module or similar to the remote.
static const uint8_t JOY_HOR_PIN        = A1;
static const uint8_t JOY_VER_PIN        = A2;

// Code below

#define XSTR(s) STR(s)
#define STR(s) #s
Protocol *protocol = nullptr;
Transmitter *tx = nullptr;

enum Axes {
  JOY_HOR_AXIS  = 0,
  JOY_VER_AXIS  = 1
};

float joyCalibHor, joyCalibVer;

bool pair() {
  protocol->discoverNodes();
  Serial.printf("%d nodes discovered\n", protocol->numDiscoveredNodes());

  if(protocol->numDiscoveredNodes() != 0) {
    for(unsigned int i=0; i<protocol->numDiscoveredNodes(); i++) {
      const NodeDescription& descr = protocol->discoveredNode(i);
      Serial.printf("    %s: %s\n", descr.addr.toString().c_str(), descr.getName().c_str());
    }

    const NodeDescription& descr = protocol->discoveredNode(0);
    Serial.printf("Trying to pair with \"%s\" (%s)... ", descr.getName().c_str(), descr.addr.toString().c_str());
    if(protocol->pairWith(descr) == true) {
      Serial.printf("success!\n");
      return true;
    }
    Serial.printf("failure.\n");
    return false;
  }

  Serial.printf("No appropriate nodes discovered!\n");
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.printf("MCS Transmitter example\n");
  Serial.printf("Transmission protocol used: %s\n", XSTR(PROTOCOL));

  joyCalibHor = 0;
  joyCalibVer = 0;
  for(unsigned int i=0; i<100; i++) {
    joyCalibHor += digitalRead(JOY_HOR_PIN) / 4096.0f;
    joyCalibVer += digitalRead(JOY_VER_PIN) / 4096.0f;
  }
  joyCalibHor /= 100.0f;
  joyCalibVer /= 100.0f;

  protocol = ProtocolFactory::createProtocol(type);
  if(protocol == nullptr) {
    Serial.printf("Error: Could not create protocol!\n");
    while(true);
  }
  protocol->init("Transmitter");
  tx = protocol->createTransmitter();
  if(tx == nullptr) {
    Serial.printf("Error: Could not create transmitter!\n");
    while(true);
  }

  while(true) {
    if(!protocol->isPaired()) pair();
    if(!protocol->isConnected()) protocol->connect();
    if(protocol->isPaired() && protocol->isConnected()) break;

    protocol->step();

    delay(1000);
  }

  tx->setAxisName(JOY_HOR_AXIS, "JoyHor");
  tx->setAxisName(JOY_VER_AXIS, "JoyVer");
}

void loop() {
  float joyVer = analogRead(JOY_VER_PIN) / 4096.0f - joyCalibVer;
  float joyHor = analogRead(JOY_HOR_PIN) / 4096.0f - joyCalibHor;
  tx->setAxisValue(JOY_VER_AXIS, joyVer, UNIT_UNITY);
  tx->setAxisValue(JOY_HOR_AXIS, joyHor, UNIT_UNITY);
  protocol->step();
  delay(10);
}