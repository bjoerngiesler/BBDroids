#if !defined(BBRDROIDDEPOTPROTOCOL_H)
#define BBRDROIDDEPOTPROTOCOL_H

#include <BLEAdvertisedDevice.h>

#include "../BBRBLEProtocol.h"

namespace bb {
namespace rmt {
class DroidDepotProtocol: public BLEProtocol {
public:
    DroidDepotProtocol();
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

#endif // #define BBRDROIDDEPOTPROTOCOL_H