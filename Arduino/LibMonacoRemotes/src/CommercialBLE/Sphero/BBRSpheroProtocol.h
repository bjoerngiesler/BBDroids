#if !defined(BBRSHPHEROPROTOCOL_H)
#define BBRSPHEROPROTOCOL_H

#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include <BLEAdvertisedDevice.h>
#include "../BBRBLEProtocol.h"

namespace bb {
namespace rmt {
class SpheroProtocol: public BLEProtocol {
public:
    SpheroProtocol();
    virtual uint8_t numTransmitterTypes();
    virtual Transmitter* createTransmitter(uint8_t transmitterType=0);
    virtual Receiver* createReceiver() { return nullptr; }
    virtual Configurator* createConfigurator() { return nullptr; }

    virtual bool pairWith(const NodeAddr& node);

    virtual bool isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice);

    virtual bool connect();

    struct Command {
        uint8_t byte1, byte2, byte3;
    };
    bool transmitCommand(const Command &cmd, uint8_t *payload, uint8_t len);

protected:
    virtual bool connect(const NodeAddr& addr);
    bool initialWrites();

    BLERemoteCharacteristic *pInitialChar_, *pControlChar_;
    uint8_t seqnum_;
};
};
};

#endif // CONFIG_IDF_TARGET_ESP32S2
#endif // #define BBRSPHEROPROTOCOL_H