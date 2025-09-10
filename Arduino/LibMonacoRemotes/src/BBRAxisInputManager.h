#if !defined(BBRAXISINPUTMANAGER_H)
#define BBRAXISINPUTMANAGER_H

#include "BBRTypes.h"
#include <map>

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
        Handling of axis input mixes

        Transmitter axes can be mapped to receiver inputs via mixing curves. Up to two axes can
        be mixed to any one input, with the mix being additive or multiplicative.
        For protocols like Monaco, the input to axis mapping is performed on the receiver side. 
        At pairing time or at startup, the transmitter asks the receiver for its set of input 
        to axis mappings. For these protocols, receiverSideMixing() should return true, and 
        syncReceiverSideMixes() should handle the synchronization.
        For protocols like DroidDepot, the receiver is closed source and has a fixed set of inputs
        and the transmitter does the mapping. For these protocols, receiverSideMixing() should
        return false, and syncReceiverSideMixes() should do nothing and return true.
    */
    virtual uint8_t numMixes();
    virtual bool hasMixForInput(uint8_t input);
    virtual bool hasMixForInput(const std::string& name) { return hasMixForInput(inputWithName(name)); }
    virtual const AxisMix& mixForInput(uint8_t index);
    virtual bool addMix(uint8_t input, const AxisMix& mix);
    virtual void clearMixes();

protected:
    std::map<uint8_t,AxisMix> mixes_;
    uint8_t lastInput_;
};

}; // rmt
}; // bb

#endif // BBRAXISINPUTMANAGER_H