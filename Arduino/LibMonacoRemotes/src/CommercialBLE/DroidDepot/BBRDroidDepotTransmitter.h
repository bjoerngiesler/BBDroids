#if !defined(BBRDROIDDEPOTTRANSMITTER_H)
#define BBRDROIDDEPOTTRANSMITTER_H

#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRTransmitter.h"

namespace bb {
namespace rmt {

class DroidDepotProtocol;

class DroidDepotTransmitter: public TransmitterBase<DroidDepotProtocol> {
public:
    DroidDepotTransmitter(DroidDepotProtocol *proto);

    bool canAddAxes() { return true; }

    virtual uint8_t numInputs();
    virtual const std::string& inputName(uint8_t input);

    virtual bool receiverSideMapping() { return false; }
    virtual bool syncReceiverSideMapping() {return true; }
    virtual Result transmit();
   
protected:
    void initialWrites();

    std::vector<uint8_t> axisValues_;
    bool connected_;
};
}; // rmt
}; // bb

#endif // !CONFIG_IDF_TARGET_ESP32S2
#endif // BBRDROIDDEPOTTRANSMITTER_H