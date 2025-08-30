#if !defined(BBRSHPHEROPROTOCOL_H)
#define BBRSPHEROPROTOCOL_H

#include <BLEAdvertisedDevice.h>

#include "../BBRBLEProtocol.h"

namespace bb {
namespace rmt {
class SpheroProtocol: public BLEProtocol {
public:
    SpheroProtocol();
    virtual uint8_t numTransmitterTypes();
    virtual uint8_t numChannels(uint8_t transmitterType);
    virtual uint8_t bitDepthForChannel(uint8_t transmitterType, uint8_t channel);

    virtual Transmitter* createTransmitter(uint8_t transmitterType=0);
    virtual Receiver* createReceiver() { return nullptr; }
    virtual Configurator* createConfigurator() { return nullptr; }

    virtual bool isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice);
};
};
};

#endif // #define BBRSPHEROPROTOCOL_H