#include <Arduino.h>
#include <WiFiNINA.h>

#include "BB8WifiServer.h"
#include "BB8Config.h"

BB8WifiServer::BB8WifiServer() {
  udp_server_started_ = false;
}

bool BB8WifiServer::tryToStartAP() {
  return WiFi.beginAP(WIFI_SSID, WIFI_WPA_KEY) == WL_AP_LISTENING;
}

bool BB8WifiServer::isAPStarted() {
  return WiFi.status() == WL_AP_LISTENING || WiFi.status() == WL_AP_CONNECTED;
}

void BB8WifiServer::startUDPServer() {
  udp_.begin(REMOTE_UDP_PORT);
  udp_server_started_ = true;
}

unsigned int BB8WifiServer::readDataIfAvailable(uint8_t *buf, unsigned int maxsize) {
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
