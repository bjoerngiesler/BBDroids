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

// PROTOCOL SECTION
// ****************

// To control Droid Depot droids, uncomment the next two lines and comment the others
#include <DroidDepotBLE/BBRProtoDDBLE.h>
#define PROTOCOL DroidDepotBLEProtocol

// To control Sphero droids, uncomment the next two lines and comment the others
// #include <SpheroBLE/BBRProtoSpheroBLE.h>
// #define PROTOCOL SpheroBLEProtocol

// To control droids running the Monaco Control System (MCS) protocol over XBee, 
// uncomment the next two lines and comment the others
// #include <MCS/XBee/BBRProtoMCSXBee.h>
// #define PROTOCOL MCSXBeeProtocol

// To control droids running the Monaco Control System (MCS) protocol over ESPnow, 
// uncomment the next two lines and comment the others
// #include <MCS/ESP/BBRProtoMCSESP.h>
// #define PROTOCOL MCSESPProtocol

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

// PINS SECTION
// ************

static const uint8_t CHANNEL_0_PIN = A0;
static const uint8_t CHANNEL_1_PIN = A1;
static const uint8_t CHANNEL_2_PIN = A2;

// Code below

#define XSTR(s) STR(s)
#define STR(s) #s

using namespace bb::rmt;
PROTOCOL protocol;
Transmitter *tx = nullptr;

bool pair() {
  if(tx == nullptr) {
    Serial.printf("No transmitter created!\n");
    return false;
  }

  protocol.discoverNodes();
  Serial.printf("%d nodes discovered\n", protocol.numDiscoveredNodes());

  if(protocol.numDiscoveredNodes() != 0) {
    for(unsigned int i=0; i<protocol.numDiscoveredNodes(); i++) {
      const bb::rmt::NodeDescription& descr = protocol.discoveredNode(i);
      Serial.printf("    %s: %s\n", descr.addr.toString().c_str(), descr.name.c_str());
    }

    NodeDescription node = protocol.discoveredNode(0);
    if(tx->pairWith(node.addr) == true) {
      Serial.printf("Successfully paired with %s(%s)!\n", node.name.c_str(), node.addr.toString().c_str());
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
  while(!Serial);
  
  Serial.printf("BBRemoteTransmitter example\n");
  Serial.printf("Transmission protocol used: %s\n", XSTR(PROTOCOL));
  
  protocol.init();
  tx = protocol.createTransmitter();
  
  if(tx == nullptr) {
    Serial.printf("Error: Could not create transmitter!\n");
    while(true);
  }
}

void loop() {
  if(!tx->isPaired()) pair();
  if(tx->requiresConnection() && !tx->isConnected()) tx->connect();

  if(tx->isPaired() == true && (tx->requiresConnection() == false || tx->isConnected() == true)) {
  }

  delay(1000);
}