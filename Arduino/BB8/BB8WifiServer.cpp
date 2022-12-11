#include <Arduino.h>
#include <WiFiNINA.h>

#include "BB8WifiServer.h"
#include "BB8Config.h"
#include "BB8ConfigStorage.h"

BB8WifiServer BB8WifiServer::server;

BB8WifiServer::BB8WifiServer() {
  udp_ = NULL;
}

BB8WifiServer::~BB8WifiServer() {
  if(udp_ != NULL) {
    WiFiUDP *temp = udp_; // eliminate race conditions on delete
    udp_ = NULL;
    delete temp;
  }
}

bool BB8WifiServer::begin() {
  int i;
  String ssid, key;
  bool ap;

  BB8ConfigStorage::storage.getWifiParams(ap, ssid, key);

  if(ap == true) {
    Serial.print("Trying to start Access Point for SSID \""); Serial.print(ssid); Serial.print("\"...");
    for(i=0; i<100; i++) {
      if(isAPStarted()) break;
      Serial.print(".");
      tryToStartAP(ssid, key);
      delay(10);
    }
  } else {
    Serial.print("Trying to connect to SSID \""); Serial.print(ssid); Serial.print("\"...");
    for(i=0; i<5; i++) {
      if(isConnected()) break;
      Serial.print(".");
      tryToConnect(ssid, key);
      delay(10);
    }
  }

  if(i==5) { 
    Serial.println("failed.");
    return false;
  } else {
    Serial.println("success.");
  }

  Serial.print("Trying to start UDP Server...");
  for(i=0; i<100; i++) {
    if(isUDPServerStarted()) break;
    Serial.print(".");
    startUDPServer();
    delay(10);
  }
  if(i==100) { 
    Serial.println("failed.");
    return false;
  } else {
    Serial.println("success.");
  }

  return true;
}

void BB8WifiServer::shutdown() {
  if(udp_) {
    udp_->stop();
    delete udp_;
    udp_ = NULL;
  }
  WiFi.end();
}

void BB8WifiServer::printStatus() {
  if(isAPStarted()) {
    Serial.print("Access point started for network '"); 
    Serial.print(WiFi.SSID());
    Serial.println("'.");
  } else if(isConnected()) {
    Serial.print("Wifi connected to network '");
    Serial.print(WiFi.SSID());
    Serial.println("'.");
  } else {
    Serial.println("Wifi not connected.");
  }

  if(NULL == udp_) Serial.println("UDP server not running.");
  else Serial.println("UDP server running.");
}

bool BB8WifiServer::tryToStartAP(const String& ssid, const String& key) {
  return WiFi.beginAP(ssid.c_str(), key.c_str()) == WL_AP_LISTENING;
}

bool BB8WifiServer::isAPStarted() {
  return WiFi.status() == WL_AP_LISTENING || WiFi.status() == WL_AP_CONNECTED;
}

bool BB8WifiServer::tryToConnect(const String& ssid, const String& key) {
  return WiFi.begin(ssid.c_str(), key.c_str()) == WL_CONNECTED;
}

bool BB8WifiServer::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void BB8WifiServer::startUDPServer() {
  if(udp_ != NULL) return; // already started
  udp_ = new WiFiUDP();
  udp_->begin(COMMAND_UDP_PORT);
}

bool BB8WifiServer::readCommandPacketIfAvailable(BB8CommandPacket &cmd, IPAddress& remoteIP) {
  unsigned int received = readDataIfAvailable((uint8_t*)(&cmd), sizeof(cmd), remoteIP);
  if(received == 0) return false;

  else if(received != sizeof(cmd)) {
    Serial.print("Wrong size packet from ");
    remoteIP.printTo(Serial);
    Serial.print(" - ");
    Serial.print(received);
    Serial.print(" instead of ");
    Serial.println(sizeof(cmd));

    return false;
  }
  return true;
}

bool BB8WifiServer::sendStatePacket(BB8StatePacket& state) {
  if(udp_ == NULL) return false;

  IPAddress localIP = WiFi.localIP();
  IPAddress broadcastIP;
  for(int i=0; i<3; i++) broadcastIP[i] = localIP[i];
  broadcastIP[3] = 255; // broadcast
    
  if(!udp_->beginPacket(broadcastIP, STATE_UDP_PORT)) {
    Serial.println("Begin packet failed!");
  }
  udp_->write((uint8_t*)(&state), sizeof(state));
  if(!udp_->endPacket()) {
    Serial.println("End packet failed in sendStatePacket()!");
    return false;
  }
  return true;
}

bool BB8WifiServer::sendCommandReply(const IPAddress& remoteIP, const uint8_t *buf, uint8_t len) {
  if(udp_ == NULL) return false;

  if(!udp_->beginPacket(remoteIP, REPLY_UDP_PORT)) {
    Serial.println("Begin packet failed!");
  }
  udp_->write(buf, len);
  if(!udp_->endPacket()) {
    Serial.println("End packet failed in sendCommandReply()!");
    return false;
  }
  return true;
}

unsigned int BB8WifiServer::readDataIfAvailable(uint8_t *buf, unsigned int maxsize, IPAddress& remoteIP) {
  if(udp_ == NULL) return 0;

  unsigned int len = udp_->parsePacket();
  if(!len) return 0;
  remoteIP = udp_->remoteIP();
  if(len > maxsize) return len;
  if(udp_->read(buf, maxsize) != len) { 
    Serial.println("Huh? Differing sizes?!"); 
    return 0;
  } else {
    return len;
  }
}
