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

class Transmitter;
class Receiver;
class Configurator;

// Abstract protocol superclass
class Protocol {
public:
    virtual bool init(const std::string& nodeName);

    virtual uint8_t numTransmitterTypes() = 0;
    virtual Transmitter* createTransmitter(uint8_t transmitterType=0);
    virtual Receiver* createReceiver();
    virtual Configurator* createConfigurator();

    virtual bool discoverNodes(float timeout = 5) = 0;
    virtual unsigned int numDiscoveredNodes();
    const NodeDescription& discoveredNode(unsigned int index);

    virtual bool pairWith(const NodeDescription& node);

    virtual bool isPaired();
    virtual bool isPaired(const NodeAddr& node);
    virtual bool acceptsPairingRequests() = 0;

    virtual bool requiresConnection() { return false; }
    virtual bool isConnected() { if(!requiresConnection()) return true; else return false; }
    virtual bool connect() { return false; }

    virtual bool step();

    const std::vector<NodeDescription>& pairedNodes() { return pairedNodes_; }
    
protected:
    virtual bool connect(const NodeAddr& addr) { return false; }

    std::vector<NodeDescription> discoveredNodes_;
    std::vector<NodeDescription> pairedNodes_;
    Transmitter* transmitter_ = nullptr;
    Receiver* receiver_ = nullptr;
    Configurator* configurator_ = nullptr;
    std::string nodeName_;
};

};
};

#endif // BBRPROTOCOL_H