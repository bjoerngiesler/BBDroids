#if !defined(BBRMESPTRANSMITTER_H)
#define BBRMESPTRANSMITTER_H

#include "../BBRTransmitter.h"
#include "BBRMProtocol.h"

namespace bb {
namespace rmt {

class MProtocol;

class MTransmitter: public TransmitterBase<MProtocol> {
public:
    MTransmitter(MProtocol* proto);
    virtual bool canAddAxes() { return false; };

    virtual uint8_t numInputs();
    virtual const std::string& inputName(uint8_t input);

    virtual Result transmit();

    virtual bool receiverSideMapping() { return true; }
    virtual bool syncReceiverSideMapping();

    virtual bool requiresConnection() { return false; }
};
}; // rmt
}; // bb

#endif // BBRMESPTRANSMITTER_H