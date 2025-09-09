#if !defined(BBRAXISINPUTMANAGER_H)
#define BBRAXISINPUTMANAGER_H

#include "BBRTypes.h"
#include <vector>

namespace bb {
namespace rmt {

class AxisInputManager {
public:
    /*
        Input handling

        Inputs are defined by the droid. For protocols like Monaco, at pairing or sync time the
        transmitter asks the receiver for any inputs it defines. For protocols like DroidDepot or
        Sphero, the number of inputs is known a priori. numInputs() is pure virtual to reflect this.
    */
    virtual uint8_t numInputs() = 0;
    virtual const std::string& inputName(uint8_t input) = 0;
    virtual uint8_t inputWithName(const std::string& name);

    /*
        Handling of axis input mappings

        Transmitter axes can be mapped to receiver inputs via mixing curves. For protocols like
        Monaco, the input to axis mapping is performed on the receiver side. At pairing time or
        at startup, the transmitter asks the receiver for its set of input to axis mappings. For these
        protocols, receiverSideAxisMapping() should return true, and syncReceiverSideAxisMapping()
        should handle the synchronization.
        For protocols like DroidDepot, the receiver is closed source and has a fixed set of inputs
        and the transmitter does the mapping. For these protocols, receiverSideAxisMapping() should
        return false, and syncReceiverSideAxisMapping() should do nothing and return true.
    */
    virtual uint8_t numMappings();
    virtual bool hasMappingForInput(uint8_t input);
    virtual bool hasMappingForInput(const std::string& name) { return hasMappingForInput(inputWithName(name)); }
    virtual const AxisInputMapping& mappingForInput(uint8_t index);
    virtual bool addMapping(const AxisInputMapping& mapping);
    virtual void clearMappings();

protected:
    std::vector<AxisInputMapping> axisInputMappings_;
};

}; // rmt
}; // bb

#endif // BBRAXISINPUTMANAGER_H