// USAGE:
// 
// All Dynamixels start with ID 1, 57600bps. This program can be used to change
// IDs interactively. Please make sure that no two servos with the same ID are
// connected at once!
//
// For caution and testing, the define SIMULATION_ONLY disables actual changes
// to be written. Comment it out to go to live mode.

#include <DynamixelShield.h>
#include <vector>
#include <LibBB.h>

#define DXL_BAUDRATE 57600
#define DXL_PROTOCOL_VERSION 2.0

#define MAXID 253
DynamixelShield dxl;
bool dxlIDsFound[MAXID+1];

bool print_help();

void setup() {
  bb::Console::console.initialize(2000000);
  bb::Runloop::runloop.initialize();
  bb::Runloop::runloop.setCycleTimeMicros(10000);
  bb::Servos::servos.initialize();

  Serial.println(); Serial.println();
  Serial.println("Dynamixel Extended Tool");
  Serial.println("=======================");

  bb::Console::console.start();
  bb::Servos::servos.start();
  bb::Runloop::runloop.start();
}

void loop() {
  // should never get here
  delay(1);
}
