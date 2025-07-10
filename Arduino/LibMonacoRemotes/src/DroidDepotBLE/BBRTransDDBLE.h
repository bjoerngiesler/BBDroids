#if !defined(BBRTRANSDDBLE_H)
#define BBRTRANSDDBLE_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRTransmitter.h"

namespace bb {
namespace rmt {
class DroidDepotBLETransmitter: public Transmitter {
public:
    DroidDepotBLETransmitter();

    virtual uint8_t numChannels();
    virtual const std::string& channelName(uint8_t channel);
    virtual uint8_t bitDepthForChannel(uint8_t channel);

    virtual void setChannelValue(uint8_t channel, float value, Unit unit);
    virtual Result transmit();
protected:
};
}; // rmt
}; // bb

#endif // BBRTRANSDDBLE_H