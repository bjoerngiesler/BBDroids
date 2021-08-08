#include <Arduino.h>
#include <WiFiNINA.h>

#include "WifiServer.h"
#include "Config.h"

WifiServer::WifiServer() {
  udp_server_started_ = false;
}

bool WifiServer::tryToStartAP() {
  return WiFi.beginAP(WIFI_SSID, WIFI_WPA_KEY) == WL_AP_LISTENING;
}

bool WifiServer::isAPStarted() {
  return WiFi.status() == WL_AP_LISTENING || WiFi.status() == WL_AP_CONNECTED;
}

void WifiServer::startUDPServer() {
  udp_.begin(REMOTE_UDP_PORT);
  udp_server_started_ = true;
}

unsigned int WifiServer::readDataIfAvailable(uint8_t *buf, unsigned int maxsize) {
  unsigned int len = udp_.parsePacket();
  if(!len) return 0;
  if(len > maxsize) return len;
  if(udp_.read(buf, maxsize) != len) { 
    Serial.println("Huh? Differing sizes?!"); 
    return 0;
  } else {
    return len;
  }
}
