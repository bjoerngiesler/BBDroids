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

#include <LibBB.h>
#include <LibBBRemotes.h>
#include <Adafruit_MCP23X17.h>

// PROTOCOL SECTION
// ****************

// To control Droid Depot droids, uncomment the next two lines and comment the others
// #include <BLE/DroidDepot/BBRDroidDepotProtocol.h>
// #define PROTOCOL DroidDepotProtocol

// To control Sphero droids, uncomment the next two lines and comment the others
// #include <BLE/Sphero/BBRSpheroProtocol.h>
// #define PROTOCOL SpheroProtocol

// To control droids running the Monaco Control System (MCS) protocol over XBee, 
// uncomment the next two lines and comment the others
// #include <MCS/XBee/BBRProtoMCSXBee.h>
// #define PROTOCOL MCSXBeeProtocol

// To control droids running the Monaco Control System (MCS) protocol over ESPnow, 
// uncomment the next two lines and comment the others
#include <MCS/ESP/BBRMESPProtocol.h>
#define PROTOCOL bb::rmt::MESPProtocol

// To control droids running the Monaco Control System (MCS) protocol over Bluetooth, 
// uncomment the next two lines and comment the others
// #include <MCS/BLE/BBRProtoMCSBLE.h>
// #define PROTOCOL MCSBLEProtocol

// To control droids running the Monaco Control System (MCS) protocol over UDP, 
// uncomment the next two lines and comment the others
// #include <MCS/UDP/BBRProtoMCSUDP.h>
// #define PROTOCOL MCSUDPProtocol

// These input pins are used to read the remote hardware. Modify them so they reflect
// how you connect your joystick module or similar to the remote.

// CONFIG SECTION
// **************

static const uint8_t JOY_VER_PIN        = A2; // A8;
static const uint8_t JOY_HOR_PIN        = A1;

// Code below

#define XSTR(s) STR(s)
#define STR(s) #s

using namespace bb::rmt;
PROTOCOL protocol;
Transmitter *tx = nullptr;

enum Axes {
  JOY_HOR_AXIS  = 0,
  JOY_VER_AXIS  = 1
};

float joyCalibHor, joyCalibVer;

bool pair() {
  protocol.discoverNodes();
  Serial.printf("%d nodes discovered\n", protocol.numDiscoveredNodes());

  if(protocol.numDiscoveredNodes() != 0) {
    for(unsigned int i=0; i<protocol.numDiscoveredNodes(); i++) {
      const bb::rmt::NodeDescription& descr = protocol.discoveredNode(i);
      Serial.printf("    %s: %s\n", descr.addr.toString().c_str(), descr.name.c_str());
    }

    NodeDescription node = protocol.discoveredNode(0);
    if(protocol.pairWith(node) == true) {
      Serial.printf("Successfully paired with \"%s\" (%s)!\n", node.name.c_str(), node.addr.toString().c_str());
      return true;
    }
    Serial.printf("Something went wrong trying to pair with %s(%s)\n", node.name.c_str(), node.addr.toString().c_str());
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

  protocol.init("Transmitter");
  tx = protocol.createTransmitter();
  if(tx == nullptr) {
    Serial.printf("Error: Could not create transmitter!\n");
    while(true);
  }

  while(true) {
    if(!protocol.isPaired()) pair();
    if(!protocol.isConnected()) protocol.connect();
    if(protocol.isPaired() && protocol.isConnected()) break;

    protocol.step();

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
  protocol.step();
  delay(10);
}