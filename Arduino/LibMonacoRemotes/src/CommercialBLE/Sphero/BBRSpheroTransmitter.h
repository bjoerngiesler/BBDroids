#if !defined(BBRSHPEROTRANSMITTER_H)
#define BBRSPHEROTRANSMITTER_H

#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRTransmitter.h"

namespace bb {
namespace rmt {

class SpheroProtocol;

class SpheroTransmitter: public TransmitterBase<SpheroProtocol> {
public:
    SpheroTransmitter(SpheroProtocol* proto);

    bool canAddAxes() { return true; }

    virtual uint8_t numInputs();
    virtual const std::string& inputName(uint8_t input);

    virtual Result transmit();

    virtual bool receiverSideMapping() { return false; }
    virtual bool syncReceiverSideMapping() { return false; }

protected:
    bool sleep(bool onoff);

    void floatToBuf(float f, uint8_t* buf);

    float lastEmote0_, lastEmote1_, lastEmote2_, lastEmote3_, lastEmote4_;
    bool connected_;
    bool sleeping_;
    float turnAngle_, domeAngle_;
};
}; // rmt
}; // bb

#endif // !CONFIG_IDF_TARGET_ESP32S2
#endif // BBRSPHEROTRANSMITTER_H