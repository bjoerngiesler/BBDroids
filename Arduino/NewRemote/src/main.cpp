#include <LibBB.h>

#include "Config.h"
#include "UI/Display.h"
#include "RemoteSubsys.h"
#include "RightSubsys.h"
#include "LeftSubsys.h"

using namespace bb;

int getAnalogReadResolution() { return 12; } // whatever

Pins pins = leftRemotePins;
bool isLeftRemote;

void setup() {
  Serial.begin(2000000);
  while(!Serial);
  
  Serial.println("BBDroids Remote Software");
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

  Serial1.setPins(pins.P_D_XBEE_RX, pins.P_D_XBEE_TX);

  if(isLeftRemote) {
    Serial2.setPins(pins.P_D_DISPLAY_RX, pins.P_D_DISPLAY_TX);
  } else {
    pinMode(pins.P_D_RST_IOEXP, OUTPUT);
    digitalWrite(pins.P_D_RST_IOEXP, HIGH);
  }

  Console::console.initialize();
  Console::console.start();

  Wire.begin();
  Wire.setClock(400000UL);

  delay(200);

  ConfigStorage::storage.initialize();
  Runloop::runloop.initialize();
  Display::display.initialize();
  RemoteSubsys::inst.initialize();
  
  if(isLeftRemote) {
    Input::inst.begin(leftRemotePins);
    LeftSubsys::inst.initialize();
  } else {
    Input::inst.begin(rightRemotePins);
    RightSubsys::inst.initialize();
  }

  Display::display.setLEDBrightness(16);  
  RemoteSubsys::inst.start();
  Display::display.start();
  if(isLeftRemote) {
    LeftSubsys::inst.start();
  } else {
    RightSubsys::inst.start();
  }

  Runloop::runloop.start(); // never returns
}


void loop() {
}