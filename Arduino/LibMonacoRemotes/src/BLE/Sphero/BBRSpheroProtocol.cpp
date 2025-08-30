#include "BBRSpheroProtocol.h"
#include "BBRSpheroTransmitter.h"

using namespace bb;
using namespace bb::rmt;

SpheroProtocol::SpheroProtocol() {
}

uint8_t SpheroProtocol::numTransmitterTypes() { return 1; }
uint8_t SpheroProtocol::numChannels(uint8_t transmitterType) { return 5; }
uint8_t SpheroProtocol::bitDepthForChannel(uint8_t transmitterType, uint8_t channel) { return 8; }

Transmitter* SpheroProtocol::createTransmitter(uint8_t transmitterType) {
    return new SpheroTransmitter();
}

bool SpheroProtocol::isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice) {
    if(advertisedDevice.getName().rfind("D2-", 0) == 0) {
        Serial.printf("Found device with name \"%s\"\n", advertisedDevice.getName().c_str());
        return true;
    }
    return false;
}
