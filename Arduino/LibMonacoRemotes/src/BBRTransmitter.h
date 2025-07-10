#if !defined(BBRTRANSMITTER_H)
#define BBRTRANSMITTER_H

#include <string>

#include <LibBB.h>

#include "BBRTypes.h"

namespace bb {
namespace rmt {

//! Abstract transmitter superclass
class Transmitter {
    virtual uint8_t numChannels() = 0;
    virtual const std::string& channelName(uint8_t channel) = 0;
    virtual uint8_t bitDepthForChannel(uint8_t channel) = 0;

    virtual void setChannelValue(uint8_t channel, float value, Unit unit) = 0;
    virtual Result transmit() = 0;
};

};
};

#endif // BBRTRANSMITTER_H