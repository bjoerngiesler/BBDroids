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

using namespace bb::rmt;

// Uncomment the protocol you want to use

// #define PROTOCOL DroidDepotProtocol
// #define PROTOCOL SpheroProtocol
// #define PROTOCOL MCSXBeeProtocol
#define PROTOCOL MESPProtocol
// #define PROTOCOL MCSBLEProtocol
// #define PROTOCOL MCSUDPProtocol

// Code below

#define XSTR(s) STR(s)
#define STR(s) #s
PROTOCOL protocol;
Receiver *rx = nullptr;

// These variables hold the values for the inputs the droid defines.
float speed=0, turn=0;
// These hold the input IDs.
uint8_t speedInput, turnInput;

// Called whenever a data packet was received.
void dataFinishedCB(const NodeAddr& addr, uint8_t seqnum) {
  Serial.printf("Speed: %f Turn: %f Seqnum: %d\n", speed, turn, seqnum);
}

// Called whenever nothing was received for a certain time (default: 0.5s).
void timeoutCB(const NodeAddr& addr) {
  Serial.printf("Timeout!\n");
  // Would stop the drive motors here
}

void setup() {
  Serial.begin(2000000);
  
  Serial.printf("MCS Receiver example\n");
  Serial.printf("Protocol used: %s\n", XSTR(PROTOCOL));

  // Initialize the protocol, giving this station a name
  protocol.init("Receiver");
  // Ask the protocol to create a receiver. Some protocols cannot do this and will return nullptr.
  rx = protocol.createReceiver();
  if(rx == nullptr) {
    Serial.printf("Error: Could not create receiver!\n");
    while(true);
  }

  // Tell the receiver about the two inputs our droid knows
  speedInput = rx->addInput(INPUT_SPEED, speed);
  turnInput = rx->addInput(INPUT_TURN_RATE, turn);

  // Tell the receiver which axis to map to which input, and how to interpolate
  rx->addMapping(AxisInputMapping(speedInput, 0, INTERP_LIN_CENTERED));
  rx->addMapping(AxisInputMapping(turnInput, 1, INTERP_LIN_CENTERED));

  // Tell the receiver to call dataFinishedCB() whenever a packet was handled
  rx->setDataFinishedCallback(dataFinishedCB);

  // Tell the receiver to call timeoutCB() whenever no data is received for a certain time
  rx->setTimeoutCallback(timeoutCB);
}

void loop() {
  // Let the protocol do its work
  protocol.step();
  delay(5);
}