#if !defined(BBRDROIDDEPOTPROTOCOL_H)
#define BBRDROIDDEPOTPROTOCOL_H

#if !CONFIG_IDF_TARGET_ESP32S2

#include <BLEAdvertisedDevice.h>

#include "../BBRBLEProtocol.h"

namespace bb {
namespace rmt {
class DroidDepotProtocol: public BLEProtocol {
public:
    DroidDepotProtocol();
    virtual uint8_t numTransmitterTypes();

    virtual Transmitter* createTransmitter(uint8_t transmitterType=0);
    virtual Receiver* createReceiver() { return nullptr; }
    virtual Configurator* createConfigurator() { return nullptr; }

    virtual bool isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice);

    virtual bool pairWith(const NodeAddr& node);

protected:
    virtual bool connect(const NodeAddr& addr);
    bool initialWrites();

    BLERemoteCharacteristic *pCharacteristic_;
}; // class DroidDepotProtocol
}; // namespace rmt
}; // namespace bb

#endif // !CONFIG_IDF_TARGET_ESP32S2

#endif // #define BBRDROIDDEPOTPROTOCOL_H