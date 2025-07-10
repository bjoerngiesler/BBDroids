#include "BBRProtoDDBLE.h"
#include "BBRTransDDBLE.h"

using namespace bb;
using namespace bb::rmt;

DroidDepotBLEProtocol::DroidDepotBLEProtocol() {
    pBLEScan_ = nullptr;
    myDevice_ = nullptr;
    pRemoteCharacteristic_ = nullptr;
}


uint8_t DroidDepotBLEProtocol::numTransmitterTypes() { return 1; }
uint8_t DroidDepotBLEProtocol::numChannels(uint8_t transmitterType) { return 5; }
uint8_t DroidDepotBLEProtocol::bitDepthForChannel(uint8_t transmitterType, uint8_t channel) { return 8; }

Transmitter* DroidDepotBLEProtocol::createTransmitter(uint8_t transmitterType) {
    return new DroidDepotBLETransmitter();
}

void DroidDepotBLEProtocol::onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Found node %s \n", advertisedDevice.toString().c_str());
//      if(advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
    if(advertisedDevice.getName() == "DROID") {
        Serial.printf("Found droid!\n");
        BLEDevice::getScan()->stop();

        esp_bd_addr_t* addr = advertisedDevice.getAddress().getNative();

        NodeDescription descr;
        descr.addr.fromMACAddress(*addr);
        descr.name = advertisedDevice.getName();
        discoveredNodes_.push_back(descr);
    }
}

Result DroidDepotBLEProtocol::discoverNodes() {
    discoveredNodes_.clear();
    pBLEScan_ = BLEDevice::getScan(); //create new scan
    pBLEScan_->setAdvertisedDeviceCallbacks(this);
    pBLEScan_->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan_->setInterval(100);
    pBLEScan_->setWindow(99);  // less or equal setInterval value
    pBLEScan_->start(5, false);
    return RES_OK;
}