#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include "BBRDroidDepotProtocol.h"
#include "BBRDroidDepotTransmitter.h"

using namespace bb;
using namespace bb::rmt;

static BLEUUID SERVICE_UUID("09b600A0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID NOTIFICATION_UUID("09b600b0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID WRITE_UUID("09b600b1-3e42-41fc-b474-e9c0c8f0c801");

DroidDepotProtocol::DroidDepotProtocol() {
    pBLEScan_ = nullptr;
    myDevice_ = nullptr;
    pCharacteristic_ = nullptr;

}

uint8_t DroidDepotProtocol::numTransmitterTypes() { return 1; }
Transmitter* DroidDepotProtocol::createTransmitter(uint8_t transmitterType) {
    if(transmitter_ == nullptr) {
        transmitter_ = new DroidDepotTransmitter(this);
    }
    return transmitter_;
}

bool DroidDepotProtocol::isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice) {
    if(advertisedDevice.getName() == "DROID") {
        Serial.printf("Found device with name \"%s\"\n", advertisedDevice.getName().c_str());
        return true;
    }
    return false;
}

bool DroidDepotProtocol::connect(const NodeAddr& addr) {
    // Connect to the remote BLE Server.
    Serial.printf("Connecting to %s\n", addr.toString().c_str());
    pClient_->connect(BLEAddress(addr.toString()), esp_ble_addr_type_t(1));
    Serial.printf(" - Connected to server\n");
    pClient_->setMTU(46); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient_->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
      Serial.printf("Failed to find our service UUID: %s\n", SERVICE_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found our service\n");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pCharacteristic_ = pRemoteService->getCharacteristic(WRITE_UUID);
    if (pCharacteristic_ == nullptr) {
      Serial.printf("Failed to find our characteristic UUID: %s\n", WRITE_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found our characteristic\n");

    initialWrites();

    return true;
}

bool DroidDepotProtocol::initialWrites() {
    if(pCharacteristic_ == nullptr) {
        Serial.printf("Characteristic is NULL\n");
        return false;
    }

    Serial.printf("Doing initial writes\n");
    for(int i=0; i<4; i++) {
        pCharacteristic_->writeValue({0x22, 0x20, 0x01}, 3);
        delay(500);
    }
    for(int i=0; i<2; i++) {
        pCharacteristic_->writeValue({0x27, 0x42, 0x0f, 0x44, 0x44, 0x00, 0x1f, 0x00}, 8);
        delay(500);
        pCharacteristic_->writeValue({0x27, 0x42, 0x0f, 0x44, 0x44, 0x00, 0x18, 0x02}, 8);
        delay(500);
    }
    Serial.printf("Initial writes done\n");

    return true;
}

bool DroidDepotProtocol::pairWith(const NodeDescription& node) {
    if(isPaired(node.addr)) {
        Serial.printf("Already paired with %s\n", node.addr.toString().c_str());
        return false;
    }

    if(pClient_ == nullptr) {
        pClient_ = BLEDevice::createClient();
        Serial.printf(" - Created client\n");

        pClient_->setClientCallbacks(this);
    }

    if(connect(node.addr) == false) {
        Serial.printf("Could not connect\n");
        return false;
    }
    
    return Protocol::pairWith(node);
}

#endif // !CONFIG_IDF_TARGET_ESP32S2