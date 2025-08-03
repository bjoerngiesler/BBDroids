#if !defined(BBRPROTOCOL_H)
#define BBRPROTOCOL_H

#include <sys/types.h>
#include <string>
#include <vector>

#include "BBRTransmitter.h"
#include "BBRReceiver.h"
#include "BBRConfigurator.h"

namespace bb {
namespace rmt {

// Abstract protocol superclass
class Protocol {
public:
    virtual bool init() = 0;

    virtual uint8_t numTransmitterTypes() = 0;
    virtual uint8_t numChannels(uint8_t transmitterType) = 0;
    virtual uint8_t bitDepthForChannel(uint8_t transmitterType, uint8_t channel) = 0;

    virtual Transmitter* createTransmitter(uint8_t transmitterType=0) { return nullptr; }
    virtual Receiver* createReceiver() { return nullptr; }
    virtual Configurator* createConfigurator() { return nullptr; }

    virtual Result discoverNodes() = 0;
    virtual unsigned int numDiscoveredNodes() { return discoveredNodes_.size(); }
    const NodeDescription& discoveredNode(unsigned int index) {
        static NodeDescription nil;
        if(index >= discoveredNodes_.size()) return nil;
        return discoveredNodes_[index];
    }
protected:
    std::vector<NodeDescription> discoveredNodes_;
};

};
};

#endif // BBRPROTOCOL_H