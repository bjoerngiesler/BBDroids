#include "BBRDroidDepotProtocol.h"
#include "BBRDroidDepotTransmitter.h"

using namespace bb;
using namespace bb::rmt;

DroidDepotProtocol::DroidDepotProtocol() {
    pBLEScan_ = nullptr;
    myDevice_ = nullptr;
    pRemoteCharacteristic_ = nullptr;
}

uint8_t DroidDepotProtocol::numTransmitterTypes() { return 1; }
uint8_t DroidDepotProtocol::numChannels(uint8_t transmitterType) { return 5; }
uint8_t DroidDepotProtocol::bitDepthForChannel(uint8_t transmitterType, uint8_t channel) { return 8; }

Transmitter* DroidDepotProtocol::createTransmitter(uint8_t transmitterType) {
    return new DroidDepotTransmitter();
}

bool DroidDepotProtocol::isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice) {
    if(advertisedDevice.getName() == "DROID") {
        Serial.printf("Found device with name \"%s\"\n", advertisedDevice.getName().c_str());
        return true;
    }
    return false;
}


