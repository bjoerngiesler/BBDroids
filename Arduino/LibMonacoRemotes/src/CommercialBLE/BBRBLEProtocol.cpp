#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include "BBRBLEProtocol.h"

using namespace bb;
using namespace bb::rmt;

BLEProtocol::BLEProtocol() {
    pBLEScan_ = nullptr;
    myDevice_ = nullptr;
}

bool BLEProtocol::init() {
    BLEDevice::init("");
    return true;
}

Result BLEProtocol::discoverNodes(float timeout) {
    discoveredNodes_.clear();
    if(pBLEScan_ == nullptr) pBLEScan_ = BLEDevice::getScan(); //create new scan
    pBLEScan_->setAdvertisedDeviceCallbacks(this);
    pBLEScan_->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan_->setInterval(100);
    pBLEScan_->setWindow(99);  // less or equal setInterval value
    pBLEScan_->start(timeout, false);
    return RES_OK;
}

void BLEProtocol::onResult(BLEAdvertisedDevice advertisedDevice) {
//      Serial.printf("Found device with name \"%s\"\n", advertisedDevice.getName().c_str());
//      if(advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
    if(isAcceptableForDiscovery(advertisedDevice)) {
        esp_bd_addr_t* addr = advertisedDevice.getAddress().getNative();
        Serial.printf("Found node %s\n", advertisedDevice.toString().c_str());
        Serial.printf("Address type 0x%x\n", advertisedDevice.getAddressType());

        NodeDescription descr;
        descr.addr.fromMACAddress(*addr);
        descr.name = advertisedDevice.getName();
        descr.isReceiver = true;
        descr.isConfigurator = false;
        descr.isTransmitter = false;
        discoveredNodes_.push_back(descr);
    }
}

void BLEProtocol::onConnect(BLEClient *pClient) {
    Serial.printf("Connected\n");
    connected_ = true;
}

void BLEProtocol::onDisconnect(BLEClient *pClient) {
    Serial.printf("Disconnected\n");
    connected_ = false;
}

#endif // !CONFIG_IDF_TARGET_ESP32S2