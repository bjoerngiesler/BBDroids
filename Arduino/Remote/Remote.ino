#include "Config.h"
#include "RemoteState.h"
#include "RemoteDisplay.h"
#include "WifiComm.h"
#include "StatePacket.h"

RemoteState *state;
RemoteDisplay *disp;
WifiComm *comm;

StatePacket packet;

unsigned long last_millis_;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BB8 Remote");
  Serial.println("Firmware Version 0.0");
  Serial.println("(c) 2022 Felix Beyer, Björn Giesler");
  Serial.println("===================================");

  disp = new RemoteDisplay;
  state = new RemoteState;
  comm = new WifiComm;

  last_millis_ = millis();

  memset(&packet, 0, sizeof(StatePacket));

  Serial.println("Entering main loop.");
}

bool connectToUDPServer() {
  int i;

  Serial.print("Trying to connect to Access Point...");
  for(i=0; i<100; i++) {
    if(comm->isConnected()) break;
    Serial.print(".");
    comm->tryToConnect();
    delay(10);
  }
  if(i==100) { 
    Serial.println("failed.");
    return false;
  } else {
    Serial.println("success.");
  }

  disp->setConnected(comm->isConnected());
  return true;
}

void runEverySecond() {
}

void loop() {
  if(millis() - last_millis_ > 1000) {
    runEverySecond();
    last_millis_ = millis();
  }

  if(comm->isConnected()) {
    state->update();
    packet.sequence_num_++;
    state->fillStatePacket(packet);
    state->printOnSerial();

    comm->broadcastUDPPacket((uint8_t*)&packet, sizeof(packet));
  } else {
    connectToUDPServer();
  }

  disp->update();

  delay(20);
}
