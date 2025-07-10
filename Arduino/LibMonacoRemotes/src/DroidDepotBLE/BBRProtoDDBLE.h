#if !defined(BBRPROTODDBLE_H)
#define BBRPROTODDBLE_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRProtocol.h"

namespace bb {
namespace rmt {
class DroidDepotBLEProtocol: public Protocol, public BLEAdvertisedDeviceCallbacks {
public:
    DroidDepotBLEProtocol();
    virtual uint8_t numTransmitterTypes();
    virtual uint8_t numChannels(uint8_t transmitterType);
    virtual uint8_t bitDepthForChannel(uint8_t transmitterType, uint8_t channel);

    virtual Transmitter* createTransmitter(uint8_t transmitterType=0);
    virtual Receiver* createReceiver() { return nullptr; }
    virtual Configurator* createConfigurator() { return nullptr; }

    virtual Result discoverNodes();

    // BLE callbacks
    void onResult(BLEAdvertisedDevice advertisedDevice);

protected:
    BLEScan* pBLEScan_;
    BLEAdvertisedDevice* myDevice_;
    BLERemoteCharacteristic* pRemoteCharacteristic_;

};
};
};

#endif // #define BBRPROTODDBLE_H