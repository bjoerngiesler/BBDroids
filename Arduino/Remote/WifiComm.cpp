#include "WifiComm.h"
#include "Config.h"

WifiComm::WifiComm() {
}

bool WifiComm::tryToConnect() {
  if(WiFi.begin(WifiSSID, WifiWPAKey) == WL_CONNECTED) {
    return true;
  }

  return false;
}

bool WifiComm::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool WifiComm::broadcastUDPPacket(uint8_t* packet, unsigned int packetsize) {
  IPAddress localIP = WiFi.localIP();
  localIP[3] = 255; // broadcast

  udp_.begin(BB8ToRemoteUDPPort);
    
  if(!udp_.beginPacket(localIP, RemoteToBB8UDPPort)) {
    Serial.println("Begin packet failed!");
  }
  udp_.write(packet, packetsize);
  if(!udp_.endPacket()) {
    Serial.println("End packet failed!");
  }
}
