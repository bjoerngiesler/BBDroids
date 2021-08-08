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

  disp = new RemoteDisplay;
  comm = new WifiComm;
  state = new RemoteState;

  last_millis_ = millis();

  memset(&packet, 0, sizeof(StatePacket));
}

void runEverySecond() {
  if(!comm->isConnected()) {
      Serial.println("Trying to connect");
      comm->tryToConnect();
      Serial.println("Done");
  } 
  
  disp->setConnected(comm->isConnected());
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

    comm->broadcastUDPPacket((uint8_t*)&packet, sizeof(packet));
  }

  disp->update();

  delay(20);
}
