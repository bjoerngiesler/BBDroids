#if !defined(BBRTRANSMITTER_H)
#define BBRTRANSMITTER_H

#include <string>
#include <vector>

#include <LibBB.h>

#include "BBRTypes.h"

namespace bb {
namespace rmt {

//! Abstract transmitter superclass
class Transmitter {
public:
    virtual uint8_t numAxes() = 0;
    virtual const std::string& axisName(uint8_t axis) = 0;
    virtual uint8_t bitDepthForAxis(uint8_t axis) = 0;

    virtual uint8_t numInputs() = 0;
    virtual const std::string& inputName(uint8_t input) = 0;
    virtual uint8_t numAxisInputMappings() { return axisInputMappings_.size(); }
    virtual const AxisInputMapping& axisInputMapping(uint8_t index) { return axisInputMappings_[index]; }
    virtual void addAxisInputMapping(const AxisInputMapping& mapping) { axisInputMappings_.push_back(mapping); }
    virtual void clearAxisInputMappings() { axisInputMappings_.clear(); }

    virtual void setAxisValue(uint8_t axis, float value, Unit unit) = 0;
    virtual Result transmit() = 0;

    virtual bool receiverSideAxisMapping() = 0;
    virtual bool syncReceiverSideAxisMapping() = 0;

    virtual bool isPaired() { return pairedNodes_.size() != 0; }
    virtual bool isPaired(const NodeAddr& node) { 
        return std::find(pairedNodes_.begin(), pairedNodes_.end(), node) != pairedNodes_.end();
    }
    virtual bool isPaired(const NodeDescription& node) { return isPaired(node.addr); }

    virtual bool pairWith(const NodeAddr& addr) = 0;
    virtual bool pairWith(const NodeDescription& node) { return pairWith(node.addr); }

    virtual bool requiresConnection() { return false; }
    virtual bool isConnected() { return false; }
    virtual bool connect() { if(pairedNodes_.size() == 0) return false; else return connect(pairedNodes_[0]); }
    virtual bool connect(const NodeAddr& addr) { return false; }

protected:
    std::vector<NodeAddr> pairedNodes_;
    std::vector<AxisInputMapping> axisInputMappings_;
};

};
};

#endif // BBRTRANSMITTER_H